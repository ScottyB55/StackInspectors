import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.figure import Figure
import numpy as np
#from py_rplidar_sdk import s2lidar
#import rplidar_sdk

#from Drone_Class import Drone, Simulated_Drone_Realistic_Physics, Simulated_Drone_Simple_Physics
import s2lidar

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

class Lidar_and_Wall_Simulator_With_GUI(tk.Tk):
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

    def __init__(self, walls, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        tk.Tk.__init__(self)
        # walls is an array of tuples of tuples
        self.walls = walls#[(wall_start_meters, wall_end_meters)]
        #self.wall_start_meters = wall_start_meters
        #self.wall_end_meters = wall_end_meters
        #print("Lidar sim Constructor: type: ", type(drone))
        self.scale_factor = 50  # Scale factor to convert meters units to pixels
        self.lidar_noise_meters_standard_dev = lidar_noise_meters_standard_dev

        self.lidar_angle_step_degrees = 2

        self.title('Drone Lidar')
        self.geometry('800x600')
        self.create_figure()
        self.update_canvas()

        # Create the Entry widget
        self.command_entry = tk.Entry(self)
        self.command_entry.pack()

        # Add a label for the Entry widget (optional)
        command_label = tk.Label(self, text="Enter command:")
        command_label.pack()

        
        import time
        import math
        from matplotlib import pyplot as plt
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
    """
    def on_command_entry_key_release(self, event):
        if event.keysym == "Return":
            command = self.command_entry.get()
            self.command_entry.delete(0, 'end')

            print(f"command received: {command}")
            if command.startswith("goto"):
                print("command starts with goto")
                try:
                    pass
                    #x, y = map(float, command[4:].split(","))
                    #drone_app.goto(x, y)
                except ValueError:
                    print("Invalid goto command")

            # Set the focus back to the canvas after processing the command
            self.focus_set()
    """
    def draw_drone(self):
        """
        Draw the drone on the matplotlib figure.
        """
        drone_center = (0, 0)  # Fixed position at the center of the screen
        drone_yaw_rad = 0 

        triangle_points = [
            (drone_center[0] + 10 * math.sin(drone_yaw_rad), drone_center[1] + 10 * math.cos(drone_yaw_rad)),
            (drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(150)), drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(150))),
            (drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(-150)), drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(-150)))
        ]

        drone_shape = plt.Polygon(triangle_points, edgecolor='blue', fill=True)
        self.ax.add_patch(drone_shape)

    def draw_walls(self):
        """
        Draw the wall on the matplotlib figure.
        """

        for wall in self.walls:
            wall_start_meters = wall.wall_start_point_relative_m
            wall_end_meters = wall.wall_end_point_relative_m 
            # wall_start_meters, wall_end_meters = wall
            self.draw_wall_from_coordinates(wall_start_meters, wall_end_meters)
    
    def draw_wall_from_coordinates(self, wall_start_meters_tuple, wall_end_meters_tuple, color = 'r-'):
        """
        Draw the wall on the matplotlib figure.
        """
        wall_start = (wall_start_meters_tuple[0] * self.scale_factor,
                      wall_start_meters_tuple[1] * self.scale_factor)
        wall_end = (wall_end_meters_tuple[0] * self.scale_factor,
                    wall_end_meters_tuple[1] * self.scale_factor)
        self.ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], color)

    def draw_point(self, point_x_m, point_y_m):
        """
        Draw a point on the matplotlib figure based on its meters coordinates.

        Args:
            point_meters (tuple): The meters coordinates of the point.
        """
        point_x = point_x_m * self.scale_factor
        point_y = point_y_m * self.scale_factor
        self.ax.plot(point_x, point_y, 'ko', markersize=1)
    """
    def draw_point_cluster(self, point_meters, label):
        cdict = {0: 'red', 1: 'blue', 2: 'green'}
        point_x = point_meters[0] * self.scale_factor
        point_y = point_meters[1] * self.scale_factor
        self.ax.plot(point_x, point_y, 'o', markersize=1, c=cdict[label])
    """
    def draw_lidar_points(self):
        """
        Draw LIDAR points on the matplotlib figure.
        """
        for lidar_reading in self.lidar_readings:
            # If lidar_distance != null
            if lidar_reading.total_relative_distance_m is not None:
                self.draw_point(lidar_reading.x_relative_distance_m, lidar_reading.y_relative_distance_m)

    """
    def draw_lidar_points_cluster(self):
        lidar_readings_absolute = []

        for lidar_angle, lidar_distance in self.lidar_readings_angle_deg_dist_m_relative:
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_readings_xy_meters_relative = lidar_reading_to_deltaxy(lidar_angle, lidar_distance)
                lidar_readings_absolute.append(lidar_readings_xy_meters_absolute)
                # self.draw_point(lidar_readings_xy_meters_absolute)
        
        from sklearn.cluster import DBSCAN
        clustering = DBSCAN(eps=3, min_samples=2).fit(lidar_readings_absolute)
        
        # print(len(lidar_readings_absolute))
        for i, point in enumerate(lidar_readings_absolute):
            self.draw_point_cluster(point, clustering.labels_[i])
    """

    def get_lidar_readings_angle_deg_dist_m(self, drone):
        """
        Returns an array of tuples containing (angle, distance) values for LIDAR readings.

        The function iterates through angles from 0 to 360 degrees, stepping by self.lidar_angle_step_degrees.
        If the LIDAR doesn't hit a wall, the distance value in the tuple is set to None.

        Angles start at 0 degrees North and move clockwise. 90 degrees is right, and 270 degrees is left relative to the drone.

        Returns:
            lidar_readings (list of tuples): A list of tuples where each tuple contains the angle (float) and the LIDAR distance (float or None).
        """
        # Pass in the reference to the drone object here! Otherwise we get an error!

        #print(f"Within update_lidar_readings about to call calc_relative_walls: ", type(drone))
        # Calculate the wall positions relative to the drone
        for wall in self.walls:
            wall.calculate_relative_walls_to_drone(drone)

        self.lidar_readings = []
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



    def get_real_lidar_readings(self, drone):
        """
        Returns an array of tuples containing (angle, distance) values for LIDAR readings from the s2lidar sensor

        The function iterates through angles from 0 to 360 degrees

        Returns:
            lidar_readings (list of tuples): A list of tuples where each tuple contains the angle (float) and the LIDAR distance (float or None).
        """
        # Pass in the reference to the drone object here! Otherwise we get an error!

        #print(f"Within update_lidar_readings about to call calc_relative_walls: ", type(drone))
        # Calculate the wall positions relative to the drone

        self.lidar_readings = []

        # while 1:
        #     distance_min = s2lidar.lidarprocess.s[0]
        #     angle = s2lidar.lidarprocess.s[1]
        #     if (distance_min != None):
        #         self.lidar_readings.append(LidarReading(angle, distance_min))

        scan = s2lidar.get_scan()

        for s in scan:
            if s[2] != 0:
                self.lidar_readings.append(LidarReading(s[0], s[1]))



    def update_canvas(self):
        """
        Update the canvas to display the latest state of the matplotlib figure.
        """
        # Create a canvas to display the figure (or update the existing canvas)
        if not hasattr(self, 'canvas'):
            self.canvas = FigureCanvasTkAgg(self.fig, master=self)
            self.canvas.get_tk_widget().pack()
        else:
            self.canvas.figure = self.fig
            self.canvas.draw()
    
    def add_text(self, text):
        self.ax.text(-390, 290, text, fontsize=12, color='black', verticalalignment='top')

    def create_figure(self):
        """
        Create a matplotlib figure to display the drone, wall, and LIDAR data.
        """
        # Create a figure
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(1, 1, 1)

        # Set the x and y axis limits
        drone_pixel_location_x = 0
        drone_pixel_location_y = 0

        # The x and y limits here keep everything relative to the position of the drone!
        self.ax.set_xlim(drone_pixel_location_x - 400,
                        drone_pixel_location_x + 400)
        self.ax.set_ylim(drone_pixel_location_y - 300,
                        drone_pixel_location_y + 300)
        
        self.x_window_min = 0 - 400 / self.scale_factor
        self.x_window_max = 0 + 400 / self.scale_factor

        # Set the x and y axis ticks
        x_ticks = np.arange(drone_pixel_location_x - 400, drone_pixel_location_x + 400, 100)
        y_ticks = np.arange(drone_pixel_location_y - 300, drone_pixel_location_y + 300, 100)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)

        # Set the x and y axis tick labels in meters units
        x_tick_labels = np.round((x_ticks - drone_pixel_location_x) / self.scale_factor, 1)
        y_tick_labels = np.round((y_ticks - drone_pixel_location_y) / self.scale_factor, 1)
        self.ax.set_xticklabels(x_tick_labels)
        self.ax.set_yticklabels(y_tick_labels)