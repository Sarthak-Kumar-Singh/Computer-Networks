from ryu.base import app_manager
from ryu.controller import ofp_event
from ryu.controller.handler import CONFIG_DISPATCHER, MAIN_DISPATCHER
from ryu.controller.handler import set_ev_cls
from ryu.ofproto import ofproto_v1_3
from ryu.lib.packet import packet
from ryu.lib.packet import ethernet
from ryu.lib.packet import ether_types

class LearningSwitch(app_manager.RyuApp):
    OFP_VERSIONS = [ofproto_v1_3.OFP_VERSION]

    def __init__(self,*args,**kwargs):
        super(LearningSwitch,self).__init__(*args,**kwargs)
        self.mac_2_port = {}

    @set_ev_cls(ofp_event.EventOFPSwitchFeatures, CONFIG_DISPATCHER)
    def switch_features_handler(self,ev):
        datapath = ev.msg.datapath
        of_version = datapath.ofproto
        parser = datapath.ofproto_parser

        match = parser.OFPMatch()
        actions = [parser.OFPActionOutput(of_version.OFPP_CONTROLLER,of_version.OFPCML_NO_BUFFER)]
        self.add_flow(datapath,0,match,actions)
        # self.logger.info("Default flow installed for switch %s", datapath.id)
        # link_list = get_all_link(self)
        # print(link_list)

    def add_flow(self,datapath,priority,match,actions):
        of_version = datapath.ofproto
        parser = datapath.ofproto_parser

        inst = [parser.OFPInstructionActions(of_version.OFPIT_APPLY_ACTIONS, actions)]
        mod = parser.OFPFlowMod(datapath=datapath, priority=priority,match=match,instructions=inst)

        datapath.send_msg(mod)
        # self.logger.info("Flow added: match=%s, actions=%s", match, actions)

    @set_ev_cls(ofp_event.EventOFPPacketIn, MAIN_DISPATCHER)
    def _packet_handler_(self,ev):
        msg = ev.msg
        datapath = msg.datapath
        of_version = datapath.ofproto
        parser = datapath.ofproto_parser
        in_port = msg.match['in_port']

        pkt = packet.Packet(msg.data)
        eth = pkt.get_protocols(ethernet.ethernet)[0]
        dst = eth.dst
        src = eth.src

        if eth.ethertype == ether_types.ETH_TYPE_LLDP:
            return

        dpid = datapath.id
        self.mac_2_port.setdefault(dpid, {})

        self.mac_2_port[dpid][src] = in_port
        # print(self.mac_2_port)
        # self.logger.info("Packet in: switch=%s, src=%s, dst=%s, in_port=%s", dpid, src, dst, in_port)

        if dst in self.mac_2_port[dpid]:
            outport = self.mac_2_port[dpid][dst]
        else:
            outport = of_version.OFPP_FLOOD
        
        actions = [parser.OFPActionOutput(outport)]

        if outport != of_version.OFPP_FLOOD:
            match = parser.OFPMatch(in_port=in_port,eth_dst = dst)
            self.add_flow(datapath,1,match,actions)
        

        out = parser.OFPPacketOut(datapath=datapath,buffer_id=msg.buffer_id,in_port=in_port,actions=actions,data=msg.data)

        datapath.send_msg(out)
        # self.logger.info("PacketOut sent: switch=%s, out_port=%s", dpid, outport)
