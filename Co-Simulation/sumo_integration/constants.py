#!/usr/bin/env python

# Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module defines constants used for the sumo-carla co-simulation. """

# ==================================================================================================
# -- constants -------------------------------------------------------------------------------------
# ==================================================================================================

INVALID_ACTOR_ID = -1
SPAWN_OFFSET_Z = 5.0  # meters
file_route_id_prefix = "file_route_"
vehicle_id_prefix = "client_vehicle_"

connect_request_keyword = "connect_request"
connect_response_keyword = "connect_response"
action_result_keyword = "action_result"
action_package_keyword = "action_package"
end_connection_keyword = "end_connection"
suspend_simulation_keyword = "suspend_simulation"
reset_simulation_keyword = "reset_simulation"
carla_id_keyword = "carla_id"
avoid_request_keyword = "avoid_request"
manual_connect_request_keyword = "manual_connect_request"
manual_connect_response_keyword = "manual_connect_response"
emergency_stop_request_keyword = "emergency_stop_request"