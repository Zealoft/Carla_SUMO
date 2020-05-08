import random
import time
import socket
import lcm
import argparse
from npc_control import initial_request, initial_response

file_route_id_prefix = "file_route_"
new_route_id_prefix = "manual_route_"
vehicle_id_prefix = "manual_vehicle_"
restart_route_id_prefix = "restart_route_"

initial_request_keyword = "initial_request"
initial_response_keyword = "initial_response"

class SUMO_Server:
    def __init__(self):
        super().__init__()
        self.vehicle_ids = []
        self.lc = lcm.LCM()
        self.lc.subscribe(initial_request_keyword, self.initial_request_handler)


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

    def server_main_loop(self):
        print("ready to listen to messages...")
        while True:
            self.lc.handle()

    
if __name__ == "__main__":
    server = SUMO_Server()
    server.server_main_loop()

    