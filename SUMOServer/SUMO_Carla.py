#!/usr/bin/python
# -*- coding: utf-8 -*-
import os
import sys
import subprocess
import time
import socket
import lcm
import argparse
from npc_control import Waypoint, action_result, connect_request, connect_response, action_package, end_connection, suspend_simulation, reset_simulation
from xml_reader import XML_Tree
from collections import deque
import threading

const_speed = 0x40
const_position = 0x42
const_2d_position = 0x00
const_angle = 0x43

const_road_id = 0x50
const_lane_id = 0x51

const_route_id = 0x53
const_edge_id = 0x54
const_lane_position = 0x56
const_best_lanes = 0xb2

file_route_id_prefix = "file_route_"
new_route_id_prefix = "manual_route_"
vehicle_id_prefix = "manual_vehicle_"
restart_route_id_prefix = "restart_route_"

connect_request_keyword = "connect_request"
connect_response_keyword = "connect_response"
action_result_keyword = "action_result"
action_package_keyword = "action_package"
end_connection_keyword = "end_connection"
suspend_simulation_keyword = "suspend_simulation"
reset_simulation_keyword = "reset_simulation"

basic_vehicle_info = (const_2d_position, const_route_id, const_edge_id, const_lane_id)

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    print("current SUMO Path is " + tools)
    sys.path.append(tools)
    print('sumo path set done')
else:
    sys.exit("please declare environmentvariable 'SUMO_HOME'")



import traci
import traci.constants as tc
import random
from sumolib.miscutils import getFreeSocketPort

print("traci import done")


class Vehicle_Client:
    def __init__(self, veh_id, rou_id):
        self.vehicle_id = veh_id
        self.route_id = rou_id
        self.simulation_steps = 0
        self.expected_simulation_steps = 0
        self.waypoint_queue = deque(maxlen=10)


class traci_simulator:
    def __init__(self, cfg_path, num_clients):
        sim_rou_path = cfg_path.split('.')[0] + '.rou.xml'
        sim_net_path = cfg_path.split('.')[0] + '.net.xml'
        self.config_file_path = cfg_path
        self.listen_port = getFreeSocketPort()
        self.num_clients = num_clients
        # 等待所有已经连接的client完成行驶后再进行下一步仿真
        self.client_events = []
        self.sumoBinary = 'sumo-gui'
        self.sumocmd = [
            self.sumoBinary, 
            "-c", 
            self.config_file_path,
            "--num-clients",
            str(num_clients)
        ]
        print(self.sumocmd)
        self.vehicle_ids = []
        rou_xml_tree = XML_Tree(sim_rou_path)
        net_xml_tree = XML_Tree(sim_net_path)
        self.routes = rou_xml_tree.read_routes()
        # new route count
        self.manual_route_num = 0
        # route count from .rou.xml file.
        self.file_route_num = 0
        self.offsets = net_xml_tree.read_offset()
        self.message_waypoints_num = 3
        # 记录所有客户端的数量，阻塞等待仿真
        self.client_num = 0
        # 阻塞等待仿真的条件变量
        self.simulation_conn = threading.Condition()
        # 记录所有已发送结果客户端序号的集合，用于判断是否得到所有客户端的结果
        # 如果在第一个客户端发送action_result时进行仿真，则判断集合是否为空
        # 如果在最后一个客户端进行仿真，则判断集合元素个数是否为已连接客户端个数
        self.client_ids = set()
        # 记录仿真进度领先的客户端id的集合链
        # 当前集合清空时如果集合链不为空，则取出集合链首部的集合作为当前集合
        self.advanced_clients_queue = deque()
        # 初始化所有SUMO子进程
        # self.init_sumo_threads()
        self.lc = lcm.LCM()
        # 用id-vehicle对来存储所有客户端
        self.vehicle_clients = {}
        # 监听需要接收的消息
        # self.lc.subscribe(connect_request_keyword, self.connect_request_handler)
        self.lc.subscribe(action_result_keyword, self.action_result_handler)
        self.lc.subscribe(suspend_simulation_keyword, self.suspend_simualtion_handler)
        self.lc.subscribe(reset_simulation_keyword, self.reset_simulation_handler)

    def init_sumo_threads(self):
        self.sumo_threads = []
        print("ready to init threads")
        for i in range(self.num_clients + 1):
            print("ready to create ", i, "# thread")
            new_thread = threading.Thread(target=self.sumo_thread_func)
            new_thread.setDaemon(True)
            new_thread.start()
            self.sumo_threads.append(new_thread)
            

    def sumo_thread_func(self):
        traci.init(self.listen_port)
        traci.setOrder(self.current_order)
        print("current order: ", self.current_order)
        self.current_order += 1
        # while True:
        #     pass

    
    # 专门接收新客户端连接消息的线程
    # 由于新客户端需要进行若干次仿真获取新车辆的位置信息，因此阻塞等待其他已连接客户端的仿真
    # 新客户端连接放在主线程中会阻塞整个程序因此新开线程，可以后续使用协程优化
    def listen_new_client_func(self):
        lc = lcm.LCM()
        lc.subscribe(connect_request_keyword, self.connect_request_handler)
        while True:
            lc.handle()
    
    """
    transform from LCM waypoint to TraCi waypoint which will be used here
    """

    def transform_LCM_to_SUMO_Waypoint(self, waypoint):
        res_point = (
            waypoint.Location[0] + self.offsets[0],
            waypoint.Location[1] + self.offsets[1]
        )
        return res_point

    """
    transform from Traci waypoint to LCM waypoint which will be used in Carla
    """

    def transform_SUMO_to_LCM_Waypoint(self, waypoint, height, angle):
        lcm_waypoint = Waypoint()
        lcm_waypoint.Location = [
            waypoint[0] - self.offsets[0],
            waypoint[1] - self.offsets[1],
            height
        ]
        lcm_waypoint.Rotation = [0.0, angle, 0.0]
        return lcm_waypoint

    def get_LCM_Waypoint(self, id):
        try:
            traci_point = traci.vehicle.getPosition(id)
            # print(traci_point)
            height = traci.vehicle.getHeight(id)
            traci_angle = traci.vehicle.getAngle(id)
            # print("angle: ", traci_angle)
            lcm_waypoint = Waypoint()

            lcm_waypoint.Location = [
                traci_point[0] - self.offsets[0],
                traci_point[1] - self.offsets[1],
                height
            ]
            lcm_waypoint.Rotation = [0.0, traci_angle, 0.0]
            return lcm_waypoint
        except traci.exceptions.TraCIException:
            # print("traci exception during getting vehicle info")
            return None
        except Exception:
            print("other error happened")
            return None

    # function called to select a random edge from file routes.
    def select_random_end_edge(self):
        rou_index = random.randint(0, self.file_route_num - 1)
        rou_id = file_route_id_prefix + str(rou_index)
        edges_list = traci.route.getEdges(rou_id)
        print("edge list: ", edges_list)
        return edges_list[len(edges_list) - 1]

    # generate a new vehicle id 
    def get_new_vehicle_id(self):
        if self.vehicle_ids.count == 0:
            new_id = vehicle_id_prefix + "0"
            self.vehicle_ids.append(new_id)
            return new_id
        else:
            maxid = -1
            for id in self.vehicle_ids:
                id_num = int(id.split('_')[2])
                if id_num > maxid:
                    maxid = id_num
            new_id = vehicle_id_prefix + str(maxid + 1)
            self.vehicle_ids.append(new_id)
            return new_id

    # function called to create a new vehichle in SUMO Server.
    # 

    # 新客户端连接时为新客户端分配车辆id、车辆路线，在仿真场景中添加新车辆，并获取初始位置并发送connect_response的方法
    # 由于此处需要执行simulationStep()，因此同样需要建立新的id集合，
    def new_vehicle_event(self):

        new_id = self.get_new_vehicle_id()
        print('new id for the client: ', new_id)
        # 从所有预设路线的前一半中随机选取一个作为初始路线
        # 尝试解决非法路线的问题
        rou_cnt = int(traci.route.getIDCount())
        print("rou count: ", rou_cnt)
        veh_rou_id = file_route_id_prefix + str(random.randint(0, rou_cnt - 1))
        print('route id for the client: ', veh_rou_id)
        # # add vehicle过程中可能出现invalid route的Exception，故不断重新选择路线进行尝试直到不抛异常
        while True:
            try:
                traci.vehicle.add(new_id, veh_rou_id)
            except Exception:
                # 重新选择路线
                veh_rou_id = file_route_id_prefix + str(random.randint(0, rou_cnt - 1))
                continue
            break
        new_vehicle = Vehicle_Client(new_id, veh_rou_id)
        self.vehicle_clients[new_id] = new_vehicle
        # @TODO 此处的simulationStep()应改为阻塞等待其他客户端的行驶完成
        # self.simulationStep()
        # 构造connect_response消息并发送
        msg = connect_response()
        msg.vehicle_id = new_id
        init_pos_way = self.get_LCM_Waypoint(new_id)
        while init_pos_way.Location[0] > 10000 or init_pos_way.Location[0] < -10000:
            self.simulationStep()
            init_pos_way = self.get_LCM_Waypoint(new_id)
        init_pos_way.Location[2] += 5
        msg.init_pos = init_pos_way
        self.lc.publish(connect_response_keyword, msg.encode())
        print("publish done. vehicle id: ", new_id)
        return new_id

    # 有客户端发来连接请求，添加车辆并进行仿真
    def connect_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = connect_request.decode(data)
        # 此处的new_vehicle_event会阻塞等待其他客户端完成当前步仿真
        id = self.new_vehicle_event()
        next_action = action_package()
        for i in range(self.message_waypoints_num):
            self.simulationStep()
            next_action.waypoints[i] = self.get_LCM_Waypoint(id)
        next_action.vehicle_id = id
        # next_action.waypoints[0] = self.transform_SUMO_to_LCM_Waypoint(id)
        # next_action.target_speed = traci.vehicle.getSpeed(id)
        self.lc.publish(action_package_keyword, next_action.encode())
        print("action package publish done!")

    # 有客户端发来行驶结果，等待所有车辆发来结果或到达最大等待时间后进行下一步仿真
    # 根据车辆id设置相应的车辆的实际状态
    def action_result_handler(self, channel, data):
        print("Received message on channel ", channel)
        # 首先解析报文信息，获取当前客户端车辆所在的位置
        msg = action_result.decode(data)
        # print("action result position: ", msg.current_pos.Location)
        res_position = self.transform_LCM_to_SUMO_Waypoint(msg.current_pos)
        is_restart = False
        try:
            print("action result position: ", res_position)
        except traci.exceptions.TraCIException:
            # 之前路线行驶完成 重新规划路线
            print("vehicle simulation ended.")
            end_pack = end_connection()
            end_pack.vehicle_id = msg.vehicle_id
            self.lc.publish(end_connection_keyword, end_pack.encode())
            return
        next_action = action_package()
        # 将客户端id加入集合或集合链的过程
        # 如果客户端id不在集合中，则将当前客户端id加入当前集合
        if msg.vehicle_id not in self.client_ids:
            self.client_ids.add(msg.vehicle_id)
        else:
            # 如果客户端id已经在当前集合中，则从集合链中找到第一个不含当前客户端id的集合
            # 如果没有这样的集合，则创建一个新集合加到list尾部
            found_set = False
            for i in range(len(self.advanced_clients_queue)):
                if msg.vehicle_id not in self.advanced_clients_queue[i]:
                    found_set = True
                    self.advanced_clients_list[i].add(msg.vehicle_id)
                    # 在集合链中的集合加满了的情况，实际上不应该出现这种情况
                    if len(self.advanced_clients_list[i]) == self.client_num:
                        print("full set in set list!")
                    break
            # 如果没有从集合链中找到符合条件的集合，则新建一个集合加入集合链
            if found_set == False:
                print("creating a new set!")
                new_set = set()
                new_set.add(msg.vehicle_id)
                self.advanced_clients_list.append(new_set)
                
        # 调用simulationStep()或从路点队列中获取将要发送的Waypoint的过程
        for i in range(self.message_waypoints_num):
            # 悲观模式：如果当前所有client都已经发送action_result报文则执行一步仿真
            # 乐观模式：当前轮第一个发送action_result的客户端执行一步仿真
            # 无论乐观或悲观模式，如果当前发送action_result的客户端已经存在在记录客户端id的集合中，则新建一个集合放在集合链的后面以记录新的一轮仿真
            # 每当执行一次simulationStep()时将当前已经填满的集合从集合链中删除
            temp_point = None
            if len(self.client_ids) == self.client_num:
                self.simulationStep(msg.vehicle_id)
                # 清空set
                self.client_ids.clear()
                temp_point = self.get_LCM_Waypoint(msg.vehicle_id)
            else:
                # 当前没有轮到执行仿真，从队列中取出路点，若队列中路点不足则考虑其他做法
                # 目前的想法是让客户端在本地临时计算下一个路点
                if len(self.vehicle_clients[msg.vehicle_id].waypoint_queue) == 0:
                    print("empty queue in vehicle_client! vehicle id: ", msg.vehicle_id)
                else:
                    temp_point = self.vehicle_clients[msg.vehicle_id].waypoint_queue.popleft()
            next_action.waypoints[i] = temp_point
            if temp_point is None:
                # 之前路线行驶完成 重新规划路线
                print("vehicle simulation ended.")
                end_pack = end_connection()
                end_pack.vehicle_id = msg.vehicle_id
                self.lc.publish(end_connection_keyword, end_pack.encode())
                return
        
        
        # 服务器端仿真完成，发送后续路点给客户端
        next_action.vehicle_id = msg.vehicle_id
        # next_action.waypoints[0] = self.transform_SUMO_to_LCM_Waypoint(msg.vehicle_id)
        # next_action.target_speed = traci.vehicle.getSpeed(msg.vehicle_id)
        self.lc.publish(action_package_keyword, next_action.encode())
        print("action package publish done!")

    def suspend_simualtion_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = suspend_simulation.decode(data)
        current_pos = self.transform_LCM_to_SUMO_Waypoint(msg.current_pos)
        print("id ", msg.vehicle_id, ": suspend simulation. current pos: ", current_pos)
        try:
            lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
            edge = traci.vehicle.getRoadID(msg.vehicle_id)
            edge = edge.split(".")[0]
            traci.vehicle.moveToXY(msg.vehicle_id, edge, lane, current_pos[0], current_pos[1], keepRoute=1)
            traci.vehicle.setStop(msg.vehicle_id, edge, until=10000)
        except traci.exceptions.TraCIException:
            print("traci exception. ")

    def reset_simulation_handler(self, channel, data):
        pass

    def simulationStep(self, veh_id):
        # 阻塞等待所有客户端都发来action_result再执行仿真
        
        for i in range(1):
            traci.simulationStep()
            for (key, value) in self.vehicle_clients.items():
                value.simulation_steps += 1
                if key != veh_id:
                    new_pos = self.get_LCM_Waypoint(key)
                    value.waypoint_queue.append(new_pos)
        

    def main_loop(self):
        traci.start(self.sumocmd, self.listen_port)
        # no need to add routes from file.
        i = 0
        for route in self.routes:
            route_name = file_route_id_prefix + str(i)
            traci.route.add(route_name, route)
            # print(traci.route.getEdges(route_name))
            i += 1
            self.file_route_num += 1
        print("begin listening LCM messages...")
        # print("find_route result: ", type(find_route_res))
        # print("traci simulation test type: ", type(traci.simulation.convertRoad(0.0, 0.0)))
        # while True:
        #     try:
        #         self.lc.handle()
        #     except KeyboardInterrupt:
        #         sys.exit()
        #         return
        while True:
            self.lc.handle()


def main():
    argparser = argparse.ArgumentParser(
        description='SUMO Simulation Client')
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=6777,
        type=int,
        help='TCP port to listen to (default: 3200)')
    argparser.add_argument(
        '-S', '--source',
        default='simulations/Town03/Town03.sumocfg',
        help='source of the sumo config file',
    )
    argparser.add_argument(
        '-n', '--num_clients',
        metavar='N',
        default=1,
        type=int,
        help='number of clients to listen to (default: 1)')
    args = argparser.parse_args()
    # sim_path = args.source.split('.')[0] + '.rou.xml'

    # print(sim_path)
    # print(args.source)
    simulator = traci_simulator(args.source, args.num_clients)
    # simulator.start_simulation()
    simulator.main_loop()


if __name__ == "__main__":
    main()
