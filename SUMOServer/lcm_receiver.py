import lcm
import sys
# sys.path.append('..')
from npc_control import Waypoint, action_result, connect_request

def connect_request_handler(channel, data):
    msg = connect_request.decode(data)
    print("Received message on channel ", channel)
    print("init pos: ", str(msg.init_pos))

lc = lcm.LCM()
subscription = lc.subscribe("connect_request", connect_request_handler)

try:
    while True:
        lc.handle()
except KeyboardInterrupt:
    pass