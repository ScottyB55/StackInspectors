import math
import numpy as np
#from py_rplidar_sdk import s2lidar
#import rplidar_sdk
#from Drone_Class import Drone, Simulated_Drone_Realistic_Physics, Simulated_Drone_Simple_Physics
import time
import json
#import py_rplidar_sdk.s2lidar as s2lidar

"""
def read_config(file_path):
    with open(file_path, "r") as file:
        config = json.load(file)
    return config

config = read_config("config.json")
lidar_type = config["lidar"]

if lidar_type == "real":
    #import real_lidar_module as lidar_module
    import py_rplidar_sdk.s2lidar as s2lidar
elif lidar_type == "simulated":
    #import simulated_lidar_module as lidar_module
    pass
else:
    raise ValueError("Invalid lidar type specified in config.json")
"""
    
class LidarReading:
    def __init__(self, angle_degrees, lidar_reading_distance_m, roll_deg=0, pitch_deg=0):
        self.lidar_angle_degrees = angle_degrees
        self.lidar_reading_distance_m = lidar_reading_distance_m
        self.roll_deg = roll_deg
        self.pitch_deg = pitch_deg
        self.update_relative_xy_distance()

    def __repr__(self):
        return f"LidarReading(angle={self.lidar_angle_degrees}, total_dist={self.lidar_reading_distance_m}, x_dist={self.x_relative_distance_m}, y_dist={self.y_relative_distance_m})"

    def update_relative_xy_distance(self):
        """
        Convert a LIDAR reading at a given angle and distance to a change in x and y coordinates.

        Args:
            lidar_angle (float): The angle of the LIDAR reading in degrees.
            distance (float): The distance of the LIDAR reading.

        Returns:
            tuple: A tuple containing the change in x and y coordinates (delta_x, delta_y).
        """
        shifted_angle = 90 - self.lidar_angle_degrees
        shifted_angle_rad = math.radians(shifted_angle)

        # TODO update this to factor in roll and pitch

        self.x_relative_distance_m = math.cos(shifted_angle_rad) * self.lidar_reading_distance_m
        self.y_relative_distance_m = math.sin(shifted_angle_rad) * self.lidar_reading_distance_m

        self.total_relative_distance_m = self.lidar_reading_distance_m
        self.angle=self.lidar_angle_degrees

class Wall:
    def __init__(self, wall_start_point_absolute_m, wall_end_point_absolute_m):
        self.wall_start_point_absolute_m = wall_start_point_absolute_m
        self.wall_end_point_absolute_m = wall_end_point_absolute_m
        self.wall_start_point_relative_m = None
        self.wall_end_point_relative_m = None

    def calculate_relative_walls_to_drone(self, drone):
        def rotate_and_translate(point, angle, translation):
            x, y = point
            x -= translation[0]
            y -= translation[1]
            angle_rad = math.radians(angle)
            x_rotated = x * math.cos(angle_rad) + y * math.sin(angle_rad)
            y_rotated = -x * math.sin(angle_rad) + y * math.cos(angle_rad)
            return x_rotated, y_rotated
        
        #print("Lidar sim calculate relative wall fn: type: ", type(drone))
        drone_yaw = drone.get_current_yaw_angle()
        drone_position = drone.drone_location_meters
        
        self.wall_start_point_relative_m = rotate_and_translate(
            self.wall_start_point_absolute_m, -drone_yaw, drone_position
        )
        self.wall_end_point_relative_m = rotate_and_translate(
            self.wall_end_point_absolute_m, -drone_yaw, drone_position
        )
        
        #self.wall_start_point_relative_m = (self.wall_start_point_absolute_m[0] - drone_position[0], self.wall_start_point_absolute_m[1] - drone_position[1])
        #self.wall_end_point_relative_m = (self.wall_end_point_absolute_m[0] - drone_position[0], self.wall_end_point_absolute_m[1] - drone_position[1])

class Lidar_and_Wall_Simulator():#tk.Tk
    """
    This class is really just a lidar & wall simulator with a GUI
    Programatically, it would have made more sense to actually have this be a separate class.
    TODO: move this class up to be the Lidar_and_Wall_Simulator_With_GUI

    Args:
        wall_start_meters (tuple): The meters coordinates of the starting point of the wall.
        wall_end_meters (tuple): The meters coordinates of the ending point of the wall.
        drone_location_meters (tuple): The meters coordinates of the drone.
        lidar_noise_meters_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self, real_lidar, walls, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        self.real_lidar = real_lidar
        #tk.Tk.__init__(self)
        # walls is an array of tuples of tuples
        self.walls = walls#[(wall_start_meters, wall_end_meters)]
        #self.wall_start_meters = wall_start_meters
        #self.wall_end_meters = wall_end_meters
        #print("Lidar sim Constructor: type: ", type(drone))
        self.scale_factor = 50  # Scale factor to convert meters units to pixels
        self.lidar_noise_meters_standard_dev = lidar_noise_meters_standard_dev

        self.lidar_angle_step_degrees = 2

        #self.title('Drone Lidar')
        #self.geometry('800x600')
        #self.create_figure()
        #self.update_canvas()

        # Create the Entry widget
        #self.command_entry = tk.Entry(self)
        #self.command_entry.pack()

        # Add a label for the Entry widget (optional)
        #command_label = tk.Label(self, text="Enter command:")
        #command_label.pack()

        if self.real_lidar == True:
            import serial.tools.list_ports
            # from pyserial import serial.tools.list_ports

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
        Returns an array of tuples containing (angle, distance) values for LIDAR readings.

        The function iterates through angles from 0 to 360 degrees, stepping by self.lidar_angle_step_degrees.
        If the LIDAR doesn't hit a wall, the distance value in the tuple is set to None.

        Angles start at 0 degrees North and move clockwise. 90 degrees is right, and 270 degrees is left relative to the drone.

        Returns:
            The time that it takes for this function to execute
        """
        # Pass in the reference to the drone object here! Otherwise we get an error!

        #print(f"Within update_lidar_readings about to call calc_relative_walls: ", type(drone))
        # Calculate the wall positions relative to the drone

        start = time.time()
        self.lidar_readings = []

        if (self.real_lidar == True):

            # while 1:
            #     distance_min = s2lidar.lidarprocess.s[0]
            #     angle = s2lidar.lidarprocess.s[1]
            #     if (distance_min != None):
            #         self.lidar_readings.append(LidarReading(angle, distance_min))

            scan = s2lidar.get_scan()

            for s in scan:
                if s[2] != 0:
                    angle=s[0]
                    self.lidar_readings.append(LidarReading(angle, s[1]/ 1000))
            
        else:
            for wall in self.walls:
                wall.calculate_relative_walls_to_drone(drone)

            angle = 0

            while angle < 360:
                angle_rad = math.radians(( - angle + 360 + 90) % 360)

                dx = math.cos(angle_rad)
                dy = math.sin(angle_rad)

                distance_min = None

                for wall in self.walls:
                    # wall_start_meters, wall_end_meters = wall

                    # Get the walls in terms of relative
                    wall_start_meters = wall.wall_start_point_relative_m

                    wall_end_meters = wall.wall_end_point_relative_m

                    t_denominator = (wall_end_meters[1] - wall_start_meters[1]) * dx - (wall_end_meters[0] - wall_start_meters[0]) * dy

                    if t_denominator != 0:
                        t_numerator = (wall_start_meters[0]) * dy - (wall_start_meters[1]) * dx
                        t = t_numerator / t_denominator

                        if not(t < 0 or t > 1):
                            u_numerator = (wall_start_meters[0]) * (wall_end_meters[1] - wall_start_meters[1]) - (wall_start_meters[1]) * (wall_end_meters[0] - wall_start_meters[0])
                            u = u_numerator / t_denominator

                            if u >= 0:
                                # The random component is from a standard normal distribution
                                # The mean distance is the lidar reading and the standard deviation is lidar_noise_meters_standard_dev
                                distance = (u + self.lidar_noise_meters_standard_dev * np.random.randn(1))[0]

                                if ((distance_min == None) or (distance < distance_min)):
                                    distance_min = distance
                
                if (distance_min != None):
                    self.lidar_readings.append(LidarReading(angle, distance_min))

                angle += self.lidar_angle_step_degrees

        end = time.time()

        return end - start

    def get_lidar_readings_angle_deg_dist_m(self):
        return self.lidar_readings
    
    def get_closest_point(self):
        # we are actually all looking at relative readings
        # drone_location_meters = self.Drone.drone_location_meters

        min_distance = None
        closest_point = None

        for lidar_reading in self.lidar_readings:
            if min_distance is None or lidar_reading.total_relative_distance_m < min_distance:
                if (not (lidar_reading.angle>50 and lidar_reading.angle<=60)) and (not (lidar_reading.angle>135 and lidar_reading.angle<=145)) and (not (lidar_reading.angle>215 and lidar_reading.angle<=225)) and (not (lidar_reading.angle>300 and lidar_reading.angle<=310)):
                    min_distance = lidar_reading.total_relative_distance_m
                    closest_point = lidar_reading
        
        return (closest_point)