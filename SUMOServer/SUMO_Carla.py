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
        self.waypoint_queue = deque(maxlen=3)


class traci_simulator:
    def __init__(self, args):
        cfg_path = args.cfg_path
        sim_rou_path = cfg_path.split('.')[0] + '.rou.xml'
        sim_net_path = cfg_path.split('.')[0] + '.net.xml'
        self.config_file_path = cfg_path
        self.listen_port = getFreeSocketPort()
        print("listen port: ", self.listen_port)
        self.num_clients = args.num_clients
        # 等待所有已经连接的client完成行驶后再进行下一步仿真
        self.client_events = []
        self.sumoBinary = 'sumo-gui'
        self.sumocmd = [
            self.sumoBinary, 
            "-c", 
            self.config_file_path,
            "--num-clients",
            str(self.num_clients),
            "--step-length",
            str(args.step_length)
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
        self.message_waypoints_num = args.message_waypoints
        self.current_order = 1
        # 记录所有客户端的数量，阻塞等待仿真
        self.client_num = 0
        # 记录所有已发送结果客户端序号的集合，用于判断是否得到所有客户端的结果
        self.client_ids = set()
        # 初始化所有SUMO子进程
        # self.init_sumo_threads()
        self.lc = lcm.LCM()
        # 用id-vehicle对来存储所有客户端
        self.vehicle_clients = {}
        # 记录所有特殊车辆id，对附近车辆做出避让命令
        # 目前的做法是存储所有id，以降低时间复杂度，id对应值为True的为特殊车辆
        self.special_vehicle_ids = {}
        # 监听需要接收的消息
        self.lc.subscribe(connect_request_keyword, self.connect_request_handler)
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

    '''
    generate a new vehicle id 
    产生符合规则的新车辆客户端id并返回
    '''
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

    '''
    在场景中产生新车辆时需要调用的方法
    获取ID、获取初始路线、构造初始位置、在场景中添加车辆、发送connect_response报文
    返回值为新车辆的id
    '''
    def new_vehicle_event(self):

        new_id = self.get_new_vehicle_id()
        print('new id for the client: ', new_id)
        # 从所有预设路线的前一半中随机选取一个作为初始路线
        # 尝试解决非法路线的问题
        rou_cnt = int(traci.route.getIDCount())
        # print("rou count: ", rou_cnt)
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
        # 将新的车辆结构体加入到dict中
        self.vehicle_clients[new_id] = new_vehicle
        self.simulationStep()
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
        msg = action_result.decode(data)
        # print("action result position: ", msg.current_pos.Location)
        res_position = self.transform_LCM_to_SUMO_Waypoint(msg.current_pos)
        is_restart = False
        # 获取当前发送action_result车辆的位置信息
        # 如果位置信息获取失败则说明车辆在SUMO场景中已经被销毁
        try:
            print("action result position: ", res_position)
            # print("current position in SUMO: ", traci.vehicle.getPosition(msg.vehicle_id))
            print("action result vehicle id: ", msg.vehicle_id)
            lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
            edge = traci.vehicle.getRoadID(msg.vehicle_id)
            edge = edge.split(".")[0]
            if traci.vehicle.isStopped(msg.vehicle_id):
                traci.vehicle.resume(msg.vehicle_id)
            # print("traci edge: ", edge)
            # print("traci lane: ", lane)
            # print("lane: ", int(lane), ", edge: ", int(edge))
        except traci.exceptions.TraCIException:
            # 之前路线行驶完成 重新规划路线
            print("vehicle simulation ended. Pick a new route")
            # print("res position is:", res_position)
            # start_edge = traci.simulation.convertRoad(res_position[0], res_position[1])[0]
            # end_edge = self.select_random_end_edge()
            # find_route_res = traci.simulation.findRoute(start_edge, end_edge)
            # edges_list = find_route_res.edges
            # rou_name = restart_route_id_prefix + str(self.manual_route_num)
            # traci.route.add(rou_name, edges_list)
            # self.manual_route_num += 1
            # traci.vehicle.add(msg.vehicle_id, rou_name)
            # lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
            # traci.vehicle.moveToXY(msg.vehicle_id, start_edge, lane, res_position[0], res_position[1], keepRoute=1)
            # self.simulationStep()
            # is_restart = True

            end_pack = end_connection()
            end_pack.vehicle_id = msg.vehicle_id
            self.lc.publish(end_connection_keyword, end_pack.encode())
            return
        # # 如果当前车辆是特殊车辆，则获取所有邻近车辆的车道信息，并判断是否需要进行换道
        # if self.special_vehicle_ids[msg.vehicle_id] is True:
        #     # 获取临近车辆信息，2代表010，即查询左侧、前侧的所有车辆
        #     neighbor_list = traci.vehicle.getNeighbors(msg.vehicle_id, 2)
        #     for (key, value) in neighbor_list:
        #         print("key: ", key, "value: ", value)
        # 获取车辆在交通仿真场景的位置并根据客户端发来的报文进行更新
        try:
            # pass
            # lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
            # edge = traci.vehicle.getRoadID(msg.vehicle_id)
            # edge = edge.split(".")[0]
            traci.vehicle.moveToXY(msg.vehicle_id, "", 0, res_position[0], res_position[1], keepRoute=1)
            # traci.vehicle.moveToXY(vehicle_id, "", 0, loc_x, loc_y, angle=yaw, keepRoute=2)
        except traci.exceptions.FatalTraCIError:
            print("traci fatal error caught")
            return
        print("position after movetoxy(): ", traci.vehicle.getPosition(msg.vehicle_id))
        # 如果当前是停车状态，则恢复行驶
        # if traci.vehicle.isStopped(msg.vehicle_id):
        #     traci.vehicle.resume(msg.vehicle_id)
        # if traci.vehicle.getSpeed(msg.vehicle_id) <= 1.0:
        #     traci.vehicle.slowDown(msg.vehicle_id, 20.0, 10000)
        next_action = action_package()
        # 调用仿真并获取车辆后续位置，发送action_package报文
        # @TODO：将每个车辆客户端均调用仿真改为第一个车辆客户端调用仿真并将其他车辆的位置信息放入队列
        for i in range(self.message_waypoints_num):
            self.simulationStep()
            # traci_point = traci.vehicle.getPosition(msg.vehicle_id)
            temp_point = self.get_LCM_Waypoint(msg.vehicle_id)
            # print("temp point: ", temp_point.Location)
            next_action.waypoints[i] = temp_point
            if temp_point is None:
                # 之前路线行驶完成 重新规划路线
                print("vehicle simulation ended. Pick a new route")
                # print("res position is:", res_position)
                # start_edge = traci.simulation.convertRoad(res_position[0], res_position[1])[0]
                # end_edge = self.select_random_end_edge()
                # find_route_res = traci.simulation.findRoute(start_edge, end_edge)
                # edges_list = find_route_res.edges
                # rou_name = restart_route_id_prefix + str(self.manual_route_num)
                # traci.route.add(rou_name, edges_list)
                # self.manual_route_num += 1
                # traci.vehicle.add(msg.vehicle_id, rou_name)
                # lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
                # traci.vehicle.moveToXY(msg.vehicle_id, start_edge, lane, res_position[0], res_position[1], keepRoute=1)
                # self.simulationStep()
                # is_restart = True
                # break
        
                end_pack = end_connection()
                end_pack.vehicle_id = msg.vehicle_id
                self.lc.publish(end_connection_keyword, end_pack.encode())
                return
        # 服务器端仿真完成，发送后续路点给客户端
        if is_restart:
            next_action_new = action_package()
            lane = traci.vehicle.getLaneIndex(msg.vehicle_id)
            traci.vehicle.moveToXY(msg.vehicle_id, start_edge, lane, res_position[0], res_position[1], keepRoute=1)
            for i in range(self.message_waypoints_num):
                self.simulationStep()
                temp_point = self.get_LCM_Waypoint(msg.vehicle_id)
                print("temp_point is ", temp_point.Location)
                next_action.waypoints[i] = temp_point
            next_action_new.vehicle_id = msg.vehicle_id
            self.lc.publish(action_package_keyword, next_action_new.encode())
        else:

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

    def simulationStep(self):
        # 阻塞等待所有客户端都发来action_result再执行仿真
        for i in range(1):
            traci.simulationStep()

    # 第一个客户端调用仿真的函数版本
    # 将其他车辆的路点放入其对应的队列中暂存
    def single_simulationStep(self, veh_id):
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
        '-C', '--cfg-path',
        default='simulations/Town03/Town03.sumocfg',
        help='source of the sumo config file',
    )
    argparser.add_argument(
        '-n', '--num-clients',
        metavar='N',
        default=1,
        type=int,
        help='number of clients to listen to (default: 1)')
    argparser.add_argument('--step-length',
        default=0.5,
        type=float,
        help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--message-waypoints',
        default=3,
        type=int,
        help='set message waypoints(default: 5)')
    args = argparser.parse_args()
    # sim_path = args.source.split('.')[0] + '.rou.xml'

    # print(sim_path)
    # print(args.source)
    simulator = traci_simulator(args)
    # simulator.start_simulation()
    simulator.main_loop()


if __name__ == "__main__":
    main()
