

import math

class DroneLidar:
    def __init__(self, wall_start, wall_end, drone_gps, drone_yaw):
        self.wall_start = wall_start
        self.wall_end = wall_end
        self.drone_gps = drone_gps
        self.drone_yaw = drone_yaw

    def update_drone_gps(self, new_gps):
        self.drone_gps = new_gps

    def update_drone_yaw(self, new_yaw):
        self.drone_yaw = new_yaw

    def get_lidar_readings(self):
        lidar_readings = [None] * 360

        for i in range(360):
            angle = (self.drone_yaw + i) % 360
            angle_rad = math.radians(angle)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            t_denominator = (self.wall_end[1] - self.wall_start[1]) * dx - (self.wall_end[0] - self.wall_start[0]) * dy

            if t_denominator == 0:
                continue

            t_numerator = (self.wall_start[0] - self.drone_gps[0]) * dy - (self.wall_start[1] - self.drone_gps[1]) * dx
            t = t_numerator / t_denominator

            if t < 0 or t > 1:
                continue

            u_numerator = (self.wall_start[0] - self.drone_gps[0]) * (self.wall_end[1] - self.wall_start[1]) - (self.wall_start[1] - self.drone_gps[1]) * (self.wall_end[0] - self.wall_start[0])
            u = u_numerator / t_denominator

            if u < 0:
                continue

            lidar_readings[i] = u

        return lidar_readings
    
    """def get_lidar_readings(self):
        lidar_readings = [None] * 360

        for i in range(360):
            angle = (self.drone_yaw + i) % 360
            angle_rad = math.radians(angle)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            t_denominator = (self.wall_end[1] - self.wall_start[1]) * dx - (self.wall_end[0] - self.wall_start[0]) * dy

            if t_denominator == 0:
                continue

            t_numerator = (self.wall_start[0] - self.drone_gps[0]) * dy - (self.wall_start[1] - self.drone_gps[1]) * dx
            t = t_numerator / t_denominator

            if t < 0 or t > 1:
                continue

            u_numerator = (self.wall_start[0] - self.drone_gps[0]) * (self.wall_end[1] - self.wall_start[1]) - (self.wall_start[1] - self.drone_gps[1]) * (self.wall_end[0] - self.wall_start[0])
            u = u_numerator / t_denominator

            if u < 0:
                continue

            intersection_x = self.wall_start[0] + t * (self.wall_end[0] - self.wall_start[0])
            intersection_y = self.wall_start[1] + t * (self.wall_end[1] - self.wall_start[1])
            distance = math.sqrt((self.drone_gps[0] - intersection_x)**2 + (self.drone_gps[1] - intersection_y)**2)
            lidar_readings[i] = distance

        return lidar_readings"""

# Example usage
wall_start = (0, 0)
wall_end = (0, 1)
drone_gps = (0.5, 0.5)
drone_yaw = 0

drone_lidar = DroneLidar(wall_start, wall_end, drone_gps, drone_yaw)
lidar_readings = drone_lidar.get_lidar_readings()

print(lidar_readings)
