#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import subprocess
import time
import lcm
import argparse
from npc_control import Waypoint, action_result, connect_request, connect_response, action_package
from xml_reader import XML_Tree

if 'SUMO_HOME' in os.environ:
    tools =os.path.join(os.environ['SUMO_HOME'], 'tools')
    print("current SUMO Path is " + tools)
    sys.path.append(tools)
    print('sumo path set done')
else:  
    sys.exit("please declare environmentvariable 'SUMO_HOME'")

import traci
import traci.constants as tc
import random
print("traci import done")


class traci_simulator:
    def __init__(self, cfg_path):
        sim_rou_path = cfg_path.split('.')[0] + '.rou.xml'
        sim_net_path = cfg_path.split('.')[0] + '.net.xml'
        self.config_file_path = cfg_path
        self.sumoBinary = 'sumo'
        self.sumocmd = [self.sumoBinary, "-c", self.config_file_path]
        self.vehicle_ids = []
        rou_xml_tree = XML_Tree(sim_rou_path)
        net_xml_tree = XML_Tree(sim_net_path)
        self.routes = rou_xml_tree.read_routes()
        self.offsets = net_xml_tree.read_offset()
        self.message_waypoints = 3
        self.lc = lcm.LCM()
        # 监听需要接收的消息
        self.lc.subscribe("connect_request", self.connect_request_handler)
        self.lc.subscribe("action_result", self.action_result_handler)