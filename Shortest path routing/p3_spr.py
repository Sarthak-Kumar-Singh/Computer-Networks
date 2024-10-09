from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet, lldp
from ryu.lib.packet import ether_types
from ryu.topology import event, switches
from ryu.topology.switches import Switches
from ryu.topology.api import get_all_switch, get_all_link
import time
from ryu.lib import mac
import struct
import heapq

class SpanningTreeSwitch(app_manager.RyuApp):
    OFP_VERSION = [ofproto_v1_3.OFP_VERSION]

    PORT_STATE_DISABLED = "disabled"
    PORT_STATE_WORKING = "working"

    def __init__(self, *args, **kwargs):
        super(SpanningTreeSwitch, self).__init__(*args, **kwargs)
        self.mac_to_port = {}
        self.network_graph = {}
        self.network_graph2 = {}
        self.spanning_tree = set()
        self.port_states = {}
        self.link_delays_starttime = {}
        self.link_delays = {}
        self.routing_table = {}
        self.total_links = 0
        self.lldp_packet_counter = 0
        self.shortest_path_algo = False
        self.count = 100
        self.host_to_switch = {}

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self, ev):
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
        self.count += 1
        self.lldp_packet_counter = 0
        self.network_graph.clear()
        switch_list = get_all_switch(self)
        link_list = dict(get_all_link(self))
        self.total_links = len(link_list)
        self.build_network_graph(switch_list, link_list)

        self.spanning_tree = self.kruskal_spanning_tree(self.network_graph)
        # self.logger.info("Spanning tree: %s", self.spanning_tree)

        self.set_port_states(link_list)
        # self.logger.info("Port States: %s", self.port_states)
        self.measure_link_delay_once(link_list)

    def build_network_graph(self, switch_list, link_list):
        for switch in switch_list:
            self.network_graph[switch.dp.id] = []

        for link in link_list:
            self.network_graph[link.src.dpid].append((link.dst.dpid, link.src.port_no))
            self.network_graph[link.dst.dpid].append((link.src.dpid, link.dst.port_no))

        # self.logger.info("Network graph: %s", self.network_graph)

    def build_network_graph_with_weights(self, switch_list, link_list):
        for switch in switch_list:
            self.network_graph2[switch.dp.id] = {}

        for link in link_list:
            if (link.src.dpid, link.dst.dpid) in self.link_delays.keys():
                self.network_graph2[link.src.dpid][link.dst.dpid] = (link.src.port_no, self.link_delays[(link.src.dpid, link.dst.dpid)])
            if (link.dst.dpid, link.src.dpid) in self.link_delays.keys():
                self.network_graph2[link.dst.dpid][link.src.dpid] = (link.dst.port_no, self.link_delays[(link.dst.dpid, link.src.dpid)])

        # self.logger.info("Network graph: %s", self.network_graph2) 
        

    def kruskal_spanning_tree(self, graph):
        # Kruskal's algorithm to construct the spanning tree
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
        return spanning_tree

    def set_port_states(self, link_list):
        for link in link_list:
            src_port_state = self.PORT_STATE_DISABLED
            dst_port_state = self.PORT_STATE_DISABLED

            if (link.src.dpid, link.dst.dpid) in self.spanning_tree or (link.dst.dpid, link.src.dpid) in self.spanning_tree:
                src_port_state = self.PORT_STATE_WORKING
                dst_port_state = self.PORT_STATE_WORKING

            self.port_states.setdefault(link.src.dpid, {})[link.src.port_no] = src_port_state
            self.port_states.setdefault(link.dst.dpid, {})[link.dst.port_no] = dst_port_state

    def measure_link_delay_once(self, link_list):
        for link in link_list:
            self.send_lldp_packet(link)

    def send_lldp_packet(self, link):
        src_dpid = link.src.dpid
        dst_dpid = link.dst.dpid
        src_port_no = link.src.port_no
        datapath = self.get_datapath_by_dpid(link.src.dpid)

        pkt = packet.Packet()
        eth = ethernet.ethernet(dst=lldp.LLDP_MAC_NEAREST_BRIDGE,
                                src=mac.BROADCAST_STR,
                                ethertype=ether_types.ETH_TYPE_LLDP)
        pkt.add_protocol(eth)

        custom_tlv = lldp.OrganizationallySpecific(
            oui=struct.pack('!I', self.count)[1:],
            subtype=1,
            info="delay551".encode('utf-8')
        )

        lldp_pkt = lldp.lldp(
            tlvs=[
                lldp.ChassisID(subtype=lldp.ChassisID.SUB_LOCALLY_ASSIGNED, chassis_id=("dpid:%016x" % src_dpid).encode('utf-8')),
                lldp.PortID(subtype=lldp.PortID.SUB_LOCALLY_ASSIGNED, port_id=str(src_port_no).encode('utf-8')),
                lldp.TTL(ttl=10),
                custom_tlv,
                lldp.End()
            ]
        )
        pkt.add_protocol(lldp_pkt)

        pkt.serialize()

        out = datapath.ofproto_parser.OFPPacketOut(
            datapath=datapath,
            buffer_id=datapath.ofproto.OFP_NO_BUFFER,
            in_port=datapath.ofproto.OFPP_CONTROLLER,
            actions=[datapath.ofproto_parser.OFPActionOutput(src_port_no)],
            data=pkt.data)
        datapath.send_msg(out)

        self.link_delays_starttime[(src_dpid, dst_dpid)] = time.time()

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
            # print("LLDP Packet came")
            lldp_pkt = pkt.get_protocol(lldp.lldp)
            
            custom_tlv_present = False
    
            for tlv in lldp_pkt.tlvs:
                if isinstance(tlv, lldp.OrganizationallySpecific) and tlv.oui == struct.pack('!I', self.count)[1:]:
                    if tlv.info.decode('utf-8') == "delay551":
                        custom_tlv_present = True
                        break

            if custom_tlv_present:
                # print("Custom LLDP packet received")
                src_dpid = datapath.id
                dst_dpid = self.get_dst_dpid_from_lldp(pkt)

                if (dst_dpid, src_dpid) in self.link_delays_starttime:
                    start_time = self.link_delays_starttime.pop((dst_dpid, src_dpid))
                    end_time = time.time()
                    delay = end_time - start_time
                    self.link_delays[(dst_dpid,src_dpid)] = delay
                    # self.logger.info("Link delay from %s to %s: %.6f seconds", dst_dpid, src_dpid, delay)
                    self.lldp_packet_counter += 1
                if self.lldp_packet_counter == self.total_links:
                    switch_list = get_all_switch(self)
                    link_list = dict(get_all_link(self))
                    self.build_network_graph_with_weights(switch_list,link_list)
                    self.routing_table = self.generate_routing_table()
                    # print(self.routing_table)
                    self.shortest_path_algo = True
            return

        dst = eth.dst
        src = eth.src

        dpid = datapath.id
        self.mac_to_port.setdefault(dpid, {})

        if src not in self.host_to_switch:
            self.host_to_switch[src] = dpid

        if self.port_states.get(dpid, {}).get(in_port) == self.PORT_STATE_DISABLED and dst not in self.host_to_switch.keys():
            return
        self.mac_to_port[dpid][src] = in_port

        if dst in self.host_to_switch.keys() and self.shortest_path_algo == True:
            dst_dpid = self.host_to_switch[dst]
            if dst_dpid != dpid:
                out_port = self.routing_table[dpid][dst_dpid]
            else:
                out_port = self.mac_to_port[dpid][dst]
        else:
            out_port = ofproto.OFPP_FLOOD

        if self.port_states.get(dpid, {}).get(out_port) == self.PORT_STATE_DISABLED and dst not in self.host_to_switch.keys():
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

    def get_dst_dpid_from_lldp(self, pkt):
        lldp_pkt = pkt.get_protocol(lldp.lldp)
        if lldp_pkt:
            for tlv in lldp_pkt.tlvs:
                if isinstance(tlv, lldp.ChassisID):
                    chassis_id_str = tlv.chassis_id.decode('utf-8')
                    if chassis_id_str.startswith('dpid:'):
                        return int(chassis_id_str.split('dpid:')[1], 16)
        return None

    def get_datapath_by_dpid(self, dpid):
        switches = get_all_switch(self)
        for switch in switches:
            if switch.dp.id == dpid:
                return switch.dp
        return None

    def dijkstra(self, source):
        distances = {switch: float('inf') for switch in self.network_graph2}
        previous_nodes = {switch: None for switch in self.network_graph2}
        distances[source] = 0
        priority_queue = [(0, source)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, (port, weight) in self.network_graph2[current_node].items():
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = (current_node, port)
                    heapq.heappush(priority_queue, (distance, neighbor))

        return previous_nodes

    def generate_routing_table(self):
        routing_table = {}

        for switch in self.network_graph2:
            previous_nodes = self.dijkstra(switch)
            routing_table[switch] = {}

            for destination in self.network_graph2:
                if destination != switch:
                    current = destination
                    while previous_nodes[current] is not None:
                        prev, port = previous_nodes[current]
                        if prev == switch:
                            routing_table[switch][destination] = port
                            break
                        current = prev

        return routing_table