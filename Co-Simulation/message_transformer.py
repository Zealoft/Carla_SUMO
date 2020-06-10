
class Message_Transformer(object):
    offsets = (0, 0)

    @staticmethod
    def transform_carla_to_lcm(carla_transform):
        lcm_waypoint = Waypoint()
        lcm_waypoint.Location = [carla_transform.location.x, -1 * carla_transform.location.y, carla_transform.location.z]
        lcm_waypoint.Rotation = [carla_transform.rotation.pitch, carla_transform.rotation.yaw, carla_transform.rotation.roll]

    @staticmethod
    def transform_lcm_to_carla(lcm_waypoint):
        """
        transfrom from LCM waypoint structure to local_planner waypoint structure.
        """
        new_waypoint = carla.libcarla.Transform()
        new_waypoint.location.x = lcm_waypoint.Location[0]
        new_waypoint.location.y = -1 * lcm_waypoint.Location[1]
        new_waypoint.location.z = lcm_waypoint.Location[2]
        new_waypoint.rotation.pitch = lcm_waypoint.Rotation[0]
        new_waypoint.rotation.yaw = lcm_waypoint.Rotation[1] - 90.0
        new_waypoint.rotation.roll = lcm_waypoint.Rotation[2]
        return new_waypoint

    @staticmethod
    def transform_LCM_to_SUMO_Waypoint(waypoint):
        res_point = (
            waypoint.Location[0] + offsets[0],
            waypoint.Location[1] + offsets[1]
        )
        return res_point