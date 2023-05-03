"""
This script provides functions and classes for reading and simulating LIDAR data.
It supports both real and simulated LIDAR data, with the ability to simulate walls and LIDAR noise.
The main classes include LidarReading, Wall, and Lidar_and_Wall_Simulator.
"""

import math
import numpy as np
import time
import json


def read_config(file_path):
    """
    Read and return the configuration data from the specified JSON file.

    Args:
        file_path (str): The path to the JSON configuration file.

    Returns:
        dict: The configuration data in a dictionary format.
    """
    with open(file_path, "r", encoding="utf-8") as file:
        config = json.load(file)
    return config


config = read_config("config.json")
use_real_lidar = config["use_real_lidar"]

if use_real_lidar == True:
    # import real lidar module
    import py_rplidar_sdk.s2lidar as s2lidar


class LidarReading:
    """
    Represents a single LIDAR reading, containing the angle, distance, roll, and pitch information.

    Attributes:
        lidar_angle_degrees (float): The angle of the LIDAR reading in degrees.
        lidar_reading_distance_m (float): The distance of the LIDAR reading in meters.
        roll_deg (float): The roll of the drone in degrees at the time of the LIDAR reading.
        pitch_deg (float): The pitch of the drone in degrees at the time of the LIDAR reading.
    """

    def __init__(
        self, angle_degrees, lidar_reading_distance_m, roll_deg=0, pitch_deg=0
    ):
        self.lidar_angle_degrees = angle_degrees
        self.lidar_reading_distance_m = lidar_reading_distance_m
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.update_relative_xy_distance()

    def __repr__(self):
        return f"LidarReading(angle={self.lidar_angle_degrees}, total_dist={self.lidar_reading_distance_m}, x_dist={self.x_relative_distance_m}, y_dist={self.y_relative_distance_m})"

    def update_relative_xy_distance(self):
        """
        Update the relative x and y distances of the LIDAR reading based on its angle and distance.
        """
        shifted_angle = 90 - self.lidar_angle_degrees
        shifted_angle_rad = math.radians(shifted_angle)

        # TODO update this to factor in roll and pitch

        self.x_relative_distance_m = (
            math.cos(shifted_angle_rad) * self.lidar_reading_distance_m
        )
        self.y_relative_distance_m = (
            math.sin(shifted_angle_rad) * self.lidar_reading_distance_m
        )

        self.total_relative_distance_m = self.lidar_reading_distance_m
        self.angle = self.lidar_angle_degrees


class Wall:
    """
    Represents a wall with a starting point and an ending point in absolute coordinates.

    Attributes:
        wall_start_point_absolute_m (tuple): The absolute coordinates of the starting point of the wall in meters.
        wall_end_point_absolute_m (tuple): The absolute coordinates of the ending point of the wall in meters.
        wall_start_point_relative_m (tuple): The relative coordinates of the starting point of the wall in meters.
        wall_end_point_relative_m (tuple): The relative coordinates of the ending point of the wall in meters.
    """

    def __init__(self, wall_start_point_absolute_m, wall_end_point_absolute_m):
        self.wall_start_point_absolute_m = wall_start_point_absolute_m
        self.wall_end_point_absolute_m = wall_end_point_absolute_m
        self.wall_start_point_relative_m = None
        self.wall_end_point_relative_m = None

    def calculate_relative_walls_to_drone(self, drone):
        """
        Calculate the relative positions of the wall's starting and ending points with respect to the drone.

        Args:
            drone (object): The drone object, which contains the drone's current position and yaw angle.
        """

        def rotate_and_translate(point, angle, translation):
            x, y = point
            x -= translation[0]
            y -= translation[1]
            angle_rad = math.radians(angle)
            x_rotated = x * math.cos(angle_rad) + y * math.sin(angle_rad)
            y_rotated = -x * math.sin(angle_rad) + y * math.cos(angle_rad)
            return x_rotated, y_rotated

        # print("Lidar sim calculate relative wall fn: type: ", type(drone))
        drone_yaw = drone.get_current_yaw_angle()
        drone_position = drone.drone_location_meters

        self.wall_start_point_relative_m = rotate_and_translate(
            self.wall_start_point_absolute_m, -drone_yaw, drone_position
        )
        self.wall_end_point_relative_m = rotate_and_translate(
            self.wall_end_point_absolute_m, -drone_yaw, drone_position
        )


class Lidar_and_Wall_Simulator:
    """
    A class to simulate LIDAR readings and walls.

    Attributes:
        walls (list): A list of Wall objects representing the walls in the environment.
        scale_factor (int): A scale factor to convert meters units to pixels.
        lidar_noise_meters_standard_dev (float): The standard deviation of the LIDAR noise in meters.
        lidar_angle_step_degrees (int): The angle step in degrees for simulated LIDAR readings.
    """

    def __init__(self, walls, lidar_noise_meters_standard_dev):  # , drone_yaw_degrees
        self.walls = walls
        self.scale_factor = 50  # Scale factor to convert meters units to pixels
        self.lidar_noise_meters_standard_dev = lidar_noise_meters_standard_dev

        self.lidar_angle_step_degrees = 2

        global use_real_lidar

        if use_real_lidar:
            import serial.tools.list_ports

            serialPortName = ""
            serialports = serial.tools.list_ports.comports()
            for port in serialports:
                if port.device.startswith("/dev/ttyUSB"):
                    serialPortName = port.device
            print(serialports, serialPortName)

            s2lidar.init(serialPortName)
            time.sleep(2)

    def read_new_lidar_readings_angle_deg_dist_m(self, drone):
        """
        Generate new LIDAR readings based on the drone's position and orientation.

        Args:
            drone (object): The drone object, which contains the drone's current position and yaw angle.

        Returns:
            float: The time it takes to execute this function.
        """

        start = time.time()
        self.lidar_readings = []

        global use_real_lidar

        if use_real_lidar:
            # use the python->c++ api to get a single scan
            scan = s2lidar.get_scan()

            # scan variable is a large list of tuples

            # where each tuple looks like this: (angle, distance, quality_flag)
            # angle:        float in degrees
            # distance:     float in millimeters
            # quality_flag: float 0-255

            # thus in below: s[0] = angle, s[1] = distance, s[2] quality_flag

            # iterate over all the tuples (points) in the scan
            for s in scan:
                # remove any points have zero quality
                if s[2] != 0:
                    # since the lidar is mounted upside down do 360 deg - angle
                    angle = 360 - s[0]
                    # convert distance to meters
                    distance = s[1] / 1000
                    self.lidar_readings.append(LidarReading(angle, distance))

        else:
            for wall in self.walls:
                wall.calculate_relative_walls_to_drone(drone)

            angle = 0

            while angle < 360:
                angle_rad = math.radians((-angle + 360 + 90) % 360)

                dx = math.cos(angle_rad)
                dy = math.sin(angle_rad)

                distance_min = None

                for wall in self.walls:
                    # wall_start_meters, wall_end_meters = wall

                    # Get the walls in terms of relative
                    wall_start_meters = wall.wall_start_point_relative_m

                    wall_end_meters = wall.wall_end_point_relative_m

                    t_denominator = (wall_end_meters[1] - wall_start_meters[1]) * dx - (
                        wall_end_meters[0] - wall_start_meters[0]
                    ) * dy

                    if t_denominator != 0:
                        t_numerator = (wall_start_meters[0]) * dy - (
                            wall_start_meters[1]
                        ) * dx
                        t = t_numerator / t_denominator

                        if not (t < 0 or t > 1):
                            u_numerator = (wall_start_meters[0]) * (
                                wall_end_meters[1] - wall_start_meters[1]
                            ) - (wall_start_meters[1]) * (
                                wall_end_meters[0] - wall_start_meters[0]
                            )
                            u = u_numerator / t_denominator

                            if u >= 0:
                                # The random component is from a standard normal distribution
                                # The mean distance is the lidar reading and the standard deviation is lidar_noise_meters_standard_dev
                                distance = (
                                    u
                                    + self.lidar_noise_meters_standard_dev
                                    * np.random.randn(1)
                                )[0]

                                if (distance_min == None) or (distance < distance_min):
                                    distance_min = distance

                if distance_min != None:
                    self.lidar_readings.append(LidarReading(angle, distance_min))

                angle += self.lidar_angle_step_degrees

        end = time.time()

        return end - start

    def get_lidar_readings_angle_deg_dist_m(self):
        """
        Get the current LIDAR readings as a list of LidarReading objects.

        Returns:
            list: The list of LidarReading objects representing the current LIDAR readings.
        """
        return self.lidar_readings

    def get_closest_point(self):
        """
        Get the closest LidarReading object from the current LIDAR readings, excluding the drone legs.

        The function iterates through the LIDAR readings and returns the reading with the smallest distance,
        ignoring readings that are closer than the specified min_distance.

        Args:
            min_distance (float): The minimum distance to consider, which is used to exclude readings of the drone legs.

        Returns:
            LidarReading: The closest LidarReading object, or None if no valid reading is found.
        """
        min_distance = None
        closest_point = None
        exclude_drone_legs_distance = 0.7

        # Cycle through the lidar readings
        for lidar_reading in self.lidar_readings:
            # Check if the distance is less than the min distance, or if it is the first point
            if (
                min_distance is None
                or lidar_reading.total_relative_distance_m < min_distance
            ):
                # Exclude nearby readings of the drone legs
                if (
                    lidar_reading.total_relative_distance_m
                    >= exclude_drone_legs_distance
                ):
                    min_distance = lidar_reading.total_relative_distance_m
                    closest_point = lidar_reading

        return closest_point
