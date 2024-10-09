from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import ether_types
from ryu.topology import event, switches
from ryu.topology.switches import Switches
from ryu.topology.api import get_all_switch, get_all_link
from ryu.lib import mac

class SpanningTreeSwitch(app_manager.RyuApp):
    OFP_VERSION = [ofproto_v1_3.OFP_VERSION]

    PORT_STATE_DISABLED = "disabled"
    PORT_STATE_WORKING = "working"

    def __init__(self, *args, **kwargs):
        super(SpanningTreeSwitch, self).__init__(*args, **kwargs)
        self.mac_to_port = {}
        self.network_graph = {}
        self.spanning_tree = set()
        self.port_states = {}

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
        # print("switch_features_handler")
        datapath = ev.msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser

        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(ofproto.OFPP_CONTROLLER, ofproto.OFPCML_NO_BUFFER)]
        self.add_flow(datapath, 0, match, actions)

    def add_flow(self, datapath, priority, match, actions, buffer_id=None):
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        inst = [parser.OFPInstructionActions(ofproto.OFPIT_APPLY_ACTIONS, actions)]
        if buffer_id:
            mod = parser.OFPFlowMod(datapath=datapath, buffer_id=buffer_id, priority=priority, match=match, instructions=inst)
        else:
            mod = parser.OFPFlowMod(datapath=datapath, priority=priority, match=match, instructions=inst)
        datapath.send_msg(mod)

    @set_ev_cls(event.EventLinkAdd)
    def get_topology_data(self, ev):
        # print("get_topology_data")
        self.network_graph.clear()
        switch_list = get_all_switch(self)
        link_list = get_all_link(self)
        # print("Switch_list", switch_list)
        # print("link_list", link_list)
        self.build_network_graph(switch_list, link_list)

        self.spanning_tree = self.kruskal_spanning_tree(self.network_graph)
        # self.logger.info("Spanning tree: %s", self.spanning_tree)

        self.set_port_states(link_list)
        # self.logger.info("Port States: %s", self.port_states)

    def build_network_graph(self, switch_list, link_list):
        for switch in switch_list:
            self.network_graph[switch.dp.id] = []

        for link in link_list:
            self.network_graph[link.src.dpid].append((link.dst.dpid, link.src.port_no))
            self.network_graph[link.dst.dpid].append((link.src.dpid, link.dst.port_no))

        # self.logger.info("Network graph: %s", self.network_graph)

    def kruskal_spanning_tree(self, graph):
        edges = []
        parent = {}
        rank = {}

        for node, neighbors in graph.items():
            for neighbor, _ in neighbors:
                if (node, neighbor) not in edges and (neighbor, node) not in edges:
                    edges.append((1, node, neighbor))

        edges = sorted(edges, key=lambda x: x[0])

        def find(node):
            if parent[node] == node:
                return node
            return find(parent[node])

        def union(node1, node2):
            root1 = find(node1)
            root2 = find(node2)
            if rank[root1] > rank[root2]:
                parent[root2] = root1
            else:
                parent[root1] = root2
                if rank[root1] == rank[root2]:
                    rank[root2] += 1

        for node in graph:
            parent[node] = node
            rank[node] = 0

        spanning_tree = set()

        for edge in edges:
            _, u, v = edge
            if find(u) != find(v):
                union(u, v)
                spanning_tree.add((u, v))
        # print(spanning_tree)
        return spanning_tree

    def set_port_states(self, link_list):
        """ Set port states as 'disabled' or 'working' based on the spanning tree. """
        for link in link_list:
            src_port_state = self.PORT_STATE_DISABLED
            dst_port_state = self.PORT_STATE_DISABLED

            if (link.src.dpid, link.dst.dpid) in self.spanning_tree or (link.dst.dpid, link.src.dpid) in self.spanning_tree:
                src_port_state = self.PORT_STATE_WORKING
                dst_port_state = self.PORT_STATE_WORKING

            self.port_states.setdefault(link.src.dpid, {})[link.src.port_no] = src_port_state
            self.port_states.setdefault(link.dst.dpid, {})[link.dst.port_no] = dst_port_state
        

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_in_handler(self, ev):
        msg = ev.msg
        datapath = msg.datapath
        ofproto = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']
        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocol(ethernet.ethernet)

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dst = eth.dst
        src = eth.src

        dpid = datapath.id
        self.mac_to_port.setdefault(dpid, {})

        if self.port_states.get(dpid, {}).get(in_port) == self.PORT_STATE_DISABLED:
            # self.logger.info("Port %s on DPID %s is not in working state. Ignoring packet.", in_port, dpid)
            return
        self.mac_to_port[dpid][src] = in_port

        # self.logger.info("Packet in DPID: %s, SRC: %s, DST: %s, IN_PORT: %s", dpid, src, dst, in_port)

        if dst in self.mac_to_port[dpid]:
            out_port = self.mac_to_port[dpid][dst]
        else:
            out_port = ofproto.OFPP_FLOOD

        if self.port_states.get(dpid, {}).get(out_port) == self.PORT_STATE_DISABLED:
            # self.logger.info("Port %s on DPID %s is not in working state. Blocking packet.", out_port, dpid)
            return

        actions = [parser.OFPActionOutput(out_port)]

        if out_port != ofproto.OFPP_FLOOD:
            match = parser.OFPMatch(in_port=in_port, eth_dst=dst, eth_src=src)
            self.add_flow(datapath, 1, match, actions)

        data = None
        if msg.buffer_id == ofproto.OFP_NO_BUFFER:
            data = msg.data

        out = parser.OFPPacketOut(
            datapath=datapath, buffer_id=msg.buffer_id, in_port=in_port,
            actions=actions, data=data)
        datapath.send_msg(out)