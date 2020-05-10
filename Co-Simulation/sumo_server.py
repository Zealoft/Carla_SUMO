import random
import time
import socket
import lcm
import argparse
import logging
import time
from npc_control import initial_request, initial_response

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
    sys.path.append('C:/Users/autolab/Desktop/0.9.8_compiled/PythonAPI/carla/dist/carla-0.9.8-py3.7-win-amd64.egg')
except IndexError:
    pass


if 'SUMO_HOME' in os.environ:
    sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")


# ==================================================================================================
# -- sumo integration importants -------------------------------------------------------------------
# ==================================================================================================

from sumo_integration.bridge_helper import BridgeHelper  # pylint: disable=wrong-import-position
from sumo_integration.carla_simulation import CarlaSimulation  # pylint: disable=wrong-import-position
from sumo_integration.constants import INVALID_ACTOR_ID  # pylint: disable=wrong-import-position
from sumo_integration.sumo_simulation import SumoSimulation  # pylint: disable=wrong-import-position





file_route_id_prefix = "file_route_"
new_route_id_prefix = "manual_route_"
vehicle_id_prefix = "manual_vehicle_"
restart_route_id_prefix = "restart_route_"

initial_request_keyword = "initial_request"
initial_response_keyword = "initial_response"

class SUMO_Server:
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.vehicle_ids = []
        self.lc = lcm.LCM()
        self.lc.subscribe(initial_request_keyword, self.initial_request_handler)
        self.carla = CarlaSimulation(args)
        self.sumo = SumoSimulation(args)

        # Mapped actor ids.
        self.sumo2carla_ids = {}  # Contains only actors controlled by sumo.
        self.carla2sumo_ids = {}  # Contains only actors controlled by carla.

        BridgeHelper.blueprint_library = self.carla.world.get_blueprint_library()
        BridgeHelper.offset = self.sumo.get_net_offset()



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

    def initial_request_handler(self, channel, data):
        print("Received message on channel ", channel)
        msg = initial_request.decode(data)
        response = initial_response()
        id = self.get_new_vehicle_id()
        print("id is ", id)
        response.vehicle_id = id
        self.lc.publish(initial_response_keyword, response.encode())

    def synchronize_vehicles(self):
        self.carla.tick()
        # Spawning new carla actors (not controlled by sumo)
        carla_spawned_actors = self.carla.spawned_actors - set(self.sumo2carla_ids.values())
        for carla_actor_id in carla_spawned_actors:
            carla_actor = self.carla.get_actor(carla_actor_id)

            type_id = BridgeHelper.get_sumo_vtype(carla_actor)
            if type_id is not None:
                # sumo_actor_id = self.sumo.spawn_actor(type_id, carla_actor.attributes)
                if sumo_actor_id != INVALID_ACTOR_ID:
                    self.carla2sumo_ids[carla_actor_id] = sumo_actor_id
                    self.sumo.subscribe(sumo_actor_id)

    def server_main_loop(self):
        print("ready to listen to messages...")
        while True:
            self.lc.handle()

    

    
if __name__ == "__main__":
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

    arguments = argparser.parse_args()

    server = SUMO_Server()
    server.server_main_loop()

    