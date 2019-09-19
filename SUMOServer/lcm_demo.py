import lcm
from npc_control import Waypoint, action_result, connect_request, connect_response, action_package, end_connection, suspend_simulation
connect_request_keyword = "connect_request"
lc = lcm.LCM()
pack = connect_request()
lc.publish(connect_request_keyword, pack.encode())