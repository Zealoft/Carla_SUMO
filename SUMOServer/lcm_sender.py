import lcm
import sys
from npc_control import Waypoint, action_result, connect_request

msg = connect_request()

lc = lcm.LCM()
lc.publish("connect_request", msg.encode())