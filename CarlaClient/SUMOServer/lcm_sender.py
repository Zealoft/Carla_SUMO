import lcm
import sys
sys.path.append('..')
from npc_control import Waypoint, action_result, connect_request

msg = connect_request()
init_point = Waypoint()
init_point.Location = [0.0, 0.0, 0.0]
msg.init_pos = init_point

lc = lcm.LCM()
lc.publish("connect_request", msg.encode())