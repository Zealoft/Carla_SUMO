#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""
Script to integrate CARLA and SUMO simulations
"""

# ==================================================================================================
# -- imports ---------------------------------------------------------------------------------------
# ==================================================================================================

import argparse
import logging
import time
import random
import threading

# ==================================================================================================
# -- find carla module -----------------------------------------------------------------------------
# ==================================================================================================

import glob
import os
import sys

try:
    # sys.path.append(glob.glob('../../PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
    #     sys.version_info.major,
    #     sys.version_info.minor,
    #     'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
    sys.path.append('C:/Users/autolab/Desktop/0.9.9.4_compiled/PythonAPI/carla/dist/carla-0.9.9-py3.7-win-amd64.egg')
except IndexError:
    pass

# ==================================================================================================
# -- find traci module -----------------------------------------------------------------------------
# ==================================================================================================

if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci

# ==================================================================================================
# -- sumo integration importants -------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID, vehicle_id_prefix, file_route_id_prefix, connect_response_keyword, connect_request_keyword, carla_id_keyword, end_connection_keyword, avoid_request_keyword, manual_connect_request_keyword, manual_connect_response_keyword, emergency_stop_request_keyword  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position

# ==================================================================================================
# -- LCM Messages --------------------------------------------------------------------------
# ==================================================================================================

from npc_control import Waypoint, action_result, connect_request, connect_response, action_package, end_connection, suspend_simulation, reset_simulation, carla_id, avoid_request, manual_connect_request, manual_connect_response, emergency_stop_request

import lcm

from queue import Queue
import queue


# ==================================================================================================
# -- synchronization_loop --------------------------------------------------------------------------
# ==================================================================================================


class SimulationSynchronization(object):
    """
    SimulationSynchronization class is responsible for the synchronization of sumo and carla
    simulations.
    """

    def __init__(self, args):
        self.args = args
        

        self.lc = lcm.LCM()

        self.sumo = SumoSimulation(args)

        

        self.carla = CarlaSimulation(args)

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.
        self.client_ids = [] # Contains agent-based clients controlled by sumo. 
        self.client_num = 0
        self.new_clients = [] # Contains new clients to be generated in carla.
        self.client_carla_ids = [] # Contains client carla ids.
        self.client_sumo_carla_ids = {}

        self.manual_client_ids = [] # Contains manual controlled client ids

        self.sumo_id_lock = threading.Lock()

        self.del_id = Queue()


        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()

        self.server_thread = threading.Thread(target=self.client_listen_process, name='ServerThread')
        self.server_thread.setDaemon(True)
        self.server_thread.start()

    def get_new_vehicle_id(self):
        if self.client_ids.count == 0:
            new_id = vehicle_id_prefix + "0"
            self.client_ids.append(new_id)
            return new_id
        else:
            maxid = -1
            for id in self.client_ids:
                id_num = int(id.split('_')[2])
                if id_num > maxid:
                    maxid = id_num
            new_id = vehicle_id_prefix + str(maxid + 1)
            self.client_ids.append(new_id)
            return new_id

    def new_vehicle_event(self):
        new_id = self.get_new_vehicle_id()
        print("new id for client is ", new_id)
        rou_cnt = self.sumo.client_route_num
        veh_rou_id = file_route_id_prefix + str(random.randint(0, rou_cnt - 1))
        while self.sumo.spawn_client_actor(new_id, veh_rou_id) == INVALID_ACTOR_ID:
            pass
        self.new_clients.append(new_id)
        

        return new_id

        
    def manual_connect_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = manual_connect_request.decode(data)
        transform = BridgeHelper.transform_LCM_to_SUMO_Waypoint(msg.init_pos)
        min_dist = -1
        min_id = None
        for id in self.carla2sumo_ids.values():
            print("sumo id: ", id)
            try:
                sumo_transform = BridgeHelper.transform_LCM_to_SUMO_Waypoint(msg.init_pos)
                (x1, y1) = (sumo_transform.location.x, sumo_transform.location.y)
                (x2, y2) = self.sumo.getPosition(id)
                print(x1, y1, x2, y2)
                dist = BridgeHelper.calc_point_square_distance(x1, y1, x2, y2)
                print("dist: ", dist)
                if min_dist == -1 or min_dist > dist:
                    min_dist = dist
                    min_id = id
            except Exception:
                print("error.")
            # print("dist: ", dist)
        print("final id for manual client: ", min_id)
        self.manual_client_ids.append(min_id)
        response = manual_connect_response()
        response.vehicle_id = min_id
        self.lc.publish(manual_connect_response_keyword, response.encode())




    def connect_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = connect_request.decode(data)
        id = self.new_vehicle_event()
        # if msg.self_controlled == False:
        #     id = self.new_vehicle_event()
        # else:
        #     pass

    def end_connection_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = end_connection.decode(data)
        veh_id = msg.vehicle_id
        try:
            # self.sumo.destroy_actor(veh_id)
            self.sumo_id_lock.acquire()
            # del self.sumo2carla_ids[veh_id]
            self.sumo_id_lock.release()
        except traci.exceptions.TraCIException:
            print("traci exception caught.")
        print("actor destroy done.")
        


    def carla_id_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = carla_id.decode(data)
        # 增加id到sumo控制的车辆中,需要加锁避免读脏数据
        self.sumo_id_lock.acquire()
        self.sumo2carla_ids[msg.vehicle_id] = msg.carla_id
        self.sumo_id_lock.release()
        print("sumo vehicle id: ", msg.vehicle_id)
        print("carla id: ", msg.carla_id)
        # 删除可能产生的多余的carla-id对
        print("carla2sumo: ", self.carla2sumo_ids[msg.carla_id])
        self.del_id.put(self.carla2sumo_ids[msg.carla_id])
        # try:
        #     self.sumo.unsubscribe(self.carla2sumo_ids[msg.carla_id])
        #     self.sumo.destroy_actor(self.carla2sumo_ids[msg.carla_id])
        # except traci.exceptions.TraCIException:
        #     print("traci exception caught.")
        # except Exception:
        #     print("caught exception")
        # for (key, value) in self.carla2sumo_ids:
        #     if value == msg.vehicle_id:
        #         print("vehicle id generated is ", self.carla2sumo_ids[key])
        #         # del self.carla2sumo_ids[key]
        #         break
        # 删除new id
        self.new_clients.remove(msg.vehicle_id)
        # self.sumo.destroy_actor(str(msg.carla_id))

    def avoid_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = avoid_request.decode(data)
        actor_id = msg.vehicle_id
        self.avoid_event(actor_id)
        # neighbors = self.sumo.getNeighbors(actor_id, mode=10)
        # print("neighbors: ", neighbors)
        # leader = self.sumo.getLeader(actor_id, dist=0.0)
        # print("leader: ", leader)

    def emergency_stop_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = emergency_stop_request.decode(data)
        actor_id = msg.vehicle_id
        try:
            traci.vehicle.slowDown(actor_id, 0.0, 120.0)
        except traci.exceptions.TraCIException:
            print("traci exception caught.")
        except TypeError:
            print("type error.")


    def avoid_event(self, start_actor_id):
        # pass
        try:
            leader = self.sumo.getLeader(start_actor_id)[0]
        except Exception:
            print("get leader error.")
            return
        except traci.exceptions.TraCIException:
            print("get leader error.")
            return
        i = 0
        while leader is not None and i < 3:
            try:
                print("leader: ", leader)
                
                if traci.vehicle.couldChangeLane(leader, -1):
                    print("could change to right")
                    traci.vehicle.changeLaneRelative(leader, -1, 60.0)
                elif traci.vehicle.couldChangeLane(leader, 1):
                    print("could change to left")
                    traci.vehicle.changeLaneRelative(leader, 1, 60.0)
                i += 1
                leader = self.sumo.getLeader(leader)[0]
            except traci.exceptions.TraCIException:
                print("traci exception caught.")
                break
            except TypeError:
                print("type error.")
                break

    # listen new client connecting requests
    def client_listen_process(self):
        print("Listening LCM Messages...")
        self.lc.subscribe(connect_request_keyword, self.connect_request_handler)
        self.lc.subscribe(carla_id_keyword, self.carla_id_handler)
        self.lc.subscribe(end_connection_keyword, self.end_connection_handler)
        self.lc.subscribe(avoid_request_keyword, self.avoid_request_handler)
        self.lc.subscribe(manual_connect_request_keyword, self.manual_connect_request_handler)
        self.lc.subscribe(emergency_stop_request_keyword, self.emergency_stop_request_handler)
        while True:
            self.lc.handle()

    def tick(self):
        """
        Tick to simulation synchronization
        """
        # -----------------
        # sumo-->carla sync
        # -----------------
        try:
            del_id = self.del_id.get(timeout=0.001)
            print("del id is ", del_id)
            # self.sumo_id_lock.acquire()
            # del_carla_id = self.sumo2carla_ids[del_id]
            # self.sumo_id_lock.release()
            # print("del carla id is ", del_carla_id)

            # self.sumo.unsubscribe(del_id)
            # self.sumo.destroy_actor(del_id)
        except queue.Empty:
            pass
        self.sumo.tick()

        # Spawning new sumo actors in carla (i.e, not controlled by carla).
        # 客户端车辆不在此处直接生成，而是通过connect_response发送报文从客户端生成
        sumo_spawned_actors = self.sumo.spawned_actors - set(self.carla2sumo_ids.values()) - set(self.new_clients)
        for sumo_actor_id in self.new_clients:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            waypoint = BridgeHelper.transform_SUMO_to_LCM_Waypoint(sumo_actor.transform)
            connect_res = connect_response()
            connect_res.init_pos = waypoint
            connect_res.vehicle_id = sumo_actor_id
            self.lc.publish(connect_response_keyword, connect_res.encode())


        for sumo_actor_id in sumo_spawned_actors:
            self.sumo.subscribe(sumo_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            carla_blueprint = BridgeHelper.get_carla_blueprint(sumo_actor,
                                                               self.args.sync_vehicle_color)
            if carla_blueprint is not None:
                carla_transform = BridgeHelper.get_carla_transform(
                    sumo_actor.transform, sumo_actor.extent)

                carla_actor_id = self.carla.spawn_actor(carla_blueprint, carla_transform)
                if carla_actor_id != INVALID_ACTOR_ID:
                    self.sumo_id_lock.acquire()
                    self.sumo2carla_ids[sumo_actor_id] = carla_actor_id
                    self.sumo_id_lock.release()
            else:
                self.sumo.unsubscribe(sumo_actor_id)

        # Destroying sumo arrived actors in carla.
        for sumo_actor_id in self.sumo.destroyed_actors:
            if sumo_actor_id in self.sumo2carla_ids:
                self.carla.destroy_actor(self.sumo2carla_ids.pop(sumo_actor_id))

        # Updating sumo actors in carla.
        self.sumo_id_lock.acquire()
        for sumo_actor_id in self.sumo2carla_ids:
            
            carla_actor_id = self.sumo2carla_ids[sumo_actor_id]
            

            sumo_actor = self.sumo.get_actor(sumo_actor_id)
            carla_actor = self.carla.get_actor(carla_actor_id)

            carla_transform = BridgeHelper.get_carla_transform(sumo_actor.transform,
                                                               sumo_actor.extent)
            if self.args.sync_vehicle_lights:
                carla_lights = BridgeHelper.get_carla_lights_state(
                    carla_actor.get_light_state(), sumo_actor.signals)
            else:
                carla_lights = None

            self.carla.synchronize_vehicle(carla_actor_id, carla_transform, carla_lights)
        self.sumo_id_lock.release()
        # -----------------
        # carla-->sumo sync
        # -----------------
        self.carla.tick()

        # Spawning new carla actors (not controlled by sumo)
        self.sumo_id_lock.acquire()
        sumo2carla_ids = self.sumo2carla_ids
        self.sumo_id_lock.release()
        carla_spawned_actors = self.carla.spawned_actors - set(sumo2carla_ids.values()) - set(self.client_carla_ids)
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            if type_id is not None:
                sumo_actor_id = self.sumo.spawn_actor(type_id, carla_actor.attributes)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

        # Destroying required carla actors in sumo.
        for carla_actor_id in self.carla.destroyed_actors:
            if carla_actor_id in self.carla2sumo_ids:
                self.sumo.destroy_actor(self.carla2sumo_ids.pop(carla_actor_id))

        # Updating carla actors in sumo.
        for carla_actor_id in self.carla2sumo_ids:
            sumo_actor_id = self.carla2sumo_ids[carla_actor_id]

            carla_actor = self.carla.get_actor(carla_actor_id)
            sumo_actor = self.sumo.get_actor(sumo_actor_id)

            sumo_transform = BridgeHelper.get_sumo_transform(carla_actor.get_transform(),
                                                             carla_actor.bounding_box.extent)
            if self.args.sync_vehicle_lights:
                carla_lights = self.carla.get_actor_light_state(carla_actor_id)
                if carla_lights is not None:
                    sumo_lights = BridgeHelper.get_sumo_lights_state(
                        sumo_actor.signals, carla_lights)
                else:
                    sumo_lights = None
            else:
                sumo_lights = None

            self.sumo.synchronize_vehicle(sumo_actor_id, sumo_transform, sumo_lights)

    def close(self):
        """
        Cleans up synchronization.
        """
        # Configuring carla simulation in async mode.
        settings = self.carla.world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        self.carla.world.apply_settings(settings)

        # Destroying synchronized actors.
        for carla_actor_id in self.sumo2carla_ids.values():
            self.carla.destroy_actor(carla_actor_id)

        for sumo_actor_id in self.carla2sumo_ids.values():
            self.sumo.destroy_actor(sumo_actor_id)

        # Closing sumo client.
        self.sumo.close()


def synchronization_loop(args):
    """
    Entry point for sumo-carla co-simulation.
    """
    synchronization = SimulationSynchronization(args)
    try:
        while True:
            start = time.time()

            synchronization.tick()

            end = time.time()
            elapsed = end - start
            if elapsed < args.step_length:
                time.sleep(args.step_length - elapsed)

    except KeyboardInterrupt:
        logging.info('Cancelled by user.')

    finally:
        logging.info('Cleaning up synchronization')

        synchronization.close()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description=__doc__)
    argparser.add_argument('--carla-host',
                           metavar='H',
                           default='127.0.0.1',
                           help='IP of the carla host server (default: 127.0.0.1)')
    argparser.add_argument('--carla-port',
                           metavar='P',
                           default=2000,
                           type=int,
                           help='TCP port to listen to (default: 2000)')
    argparser.add_argument('--sumo-host',
                           metavar='H',
                           default=None,
                           help='IP of the sumo host server (default: 127.0.0.1)')
    argparser.add_argument('--sumo-port',
                           metavar='P',
                           default=None,
                           type=int,
                           help='TCP port to liston to (default: 8813)')
    argparser.add_argument('-c',
                           '--sumo-cfg-file',
                           default=None,
                           type=str,
                           help='sumo configuration file')
    argparser.add_argument('--sumo-gui',
                           default=True,
                           help='run the gui version of sumo (default: True)')
    argparser.add_argument('--step-length',
                           default=0.05,
                           type=float,
                           help='set fixed delta seconds (default: 0.05s)')
    argparser.add_argument('--sync-vehicle-lights',
                           action='store_true',
                           help='synchronize vehicle lights state (default: False)')
    argparser.add_argument('--sync-vehicle-color',
                           action='store_true',
                           help='synchronize vehicle color (default: False)')
    argparser.add_argument('--sync-all',
                           action='store_true',
                           help='synchronize all vehicle properties (default: False)')
    argparser.add_argument('--debug', action='store_true', help='enable debug messages')
    arguments = argparser.parse_args()

    if arguments.sync_all is True:
        arguments.sync_vehicle_lights = True
        arguments.sync_vehicle_color = True

    if arguments.debug:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.DEBUG)
    else:
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

    synchronization_loop(arguments)
