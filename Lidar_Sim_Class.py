"""
TODO: plot the actual line & the line reconstructed with the LIDAR (different colors on the same plot)
TODO: add noise to the LIDAR samples
TODO: interface with the Guided_Drone_Class (See Drone_Instance.py as a starting point template)

See High Level Notes
https://docs.google.com/document/d/12D82uqbBmeTQVgK-HZTM1lzcWsRPCvrDDjQVxNA4sWw/edit#
"""

import math

class DroneLidar:
    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, drone_yaw_degrees):
        self.wall_start_gps = wall_start_gps
        self.wall_end_gps = wall_end_gps
        self.drone_gps = drone_gps
        self.drone_yaw_degrees = drone_yaw_degrees

    def update_drone_gps(self, new_gps):
        self.drone_gps = new_gps

    def update_drone_yaw_degrees(self, new_yaw):
        self.drone_yaw_degrees = new_yaw

    def get_lidar_readings_gps(self):
        lidar_readings_gps = [None] * 360

        for i in range(360):
            angle = (self.drone_yaw_degrees + i) % 360
            angle_rad = math.radians(angle)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            t_denominator = (self.wall_end_gps[1] - self.wall_start_gps[1]) * dx - (self.wall_end_gps[0] - self.wall_start_gps[0]) * dy

            if t_denominator == 0:
                continue

            t_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * dy - (self.wall_start_gps[1] - self.drone_gps[1]) * dx
            t = t_numerator / t_denominator

            if t < 0 or t > 1:
                continue

            u_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * (self.wall_end_gps[1] - self.wall_start_gps[1]) - (self.wall_start_gps[1] - self.drone_gps[1]) * (self.wall_end_gps[0] - self.wall_start_gps[0])
            u = u_numerator / t_denominator

            if u < 0:
                continue

            lidar_readings_gps[i] = u

        return lidar_readings_gps
    
    # Another function implementation
    """def get_lidar_readings_gps(self):
        lidar_readings_gps = [None] * 360

        for i in range(360):
            angle = (self.drone_yaw_degrees + i) % 360
            angle_rad = math.radians(angle)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            t_denominator = (self.wall_end_gps[1] - self.wall_start_gps[1]) * dx - (self.wall_end_gps[0] - self.wall_start_gps[0]) * dy

            if t_denominator == 0:
                continue

            t_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * dy - (self.wall_start_gps[1] - self.drone_gps[1]) * dx
            t = t_numerator / t_denominator

            if t < 0 or t > 1:
                continue

            u_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * (self.wall_end_gps[1] - self.wall_start_gps[1]) - (self.wall_start_gps[1] - self.drone_gps[1]) * (self.wall_end_gps[0] - self.wall_start_gps[0])
            u = u_numerator / t_denominator

            if u < 0:
                continue

            intersection_x = self.wall_start_gps[0] + t * (self.wall_end_gps[0] - self.wall_start_gps[0])
            intersection_y = self.wall_start_gps[1] + t * (self.wall_end_gps[1] - self.wall_start_gps[1])
            distance = math.sqrt((self.drone_gps[0] - intersection_x)**2 + (self.drone_gps[1] - intersection_y)**2)
            lidar_readings_gps[i] = distance

        return lidar_readings_gps"""

# Example usage
wall_start_gps = (0, 0)
wall_end_gps = (0, 1)
drone_gps = (0.5, 0.5)
drone_yaw_degrees = 0

drone_lidar = DroneLidar(wall_start_gps, wall_end_gps, drone_gps, drone_yaw_degrees)
lidar_readings_gps = drone_lidar.get_lidar_readings_gps()

print(lidar_readings_gps)
