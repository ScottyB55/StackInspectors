import math
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import tkinter as tk
from matplotlib.figure import Figure
import numpy as np

def lidar_reading_to_deltaxy(lidar_angle, distance):
    """
    Convert a LIDAR reading at a given angle and distance to a change in x and y coordinates.

    Args:
        lidar_angle (float): The angle of the LIDAR reading in degrees.
        distance (float): The distance of the LIDAR reading.

    Returns:
        tuple: A tuple containing the change in x and y coordinates (delta_x, delta_y).
    """
    shifted_angle = 90 - lidar_angle
    shifted_angle_rad = math.radians(shifted_angle)

    delta_x = math.cos(shifted_angle_rad) * distance
    delta_y = math.sin(shifted_angle_rad) * distance

    return delta_x, delta_y

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

    def __init__(self, walls, drone_location_meters, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        tk.Tk.__init__(self)
        # walls is an array of tuples of tuples
        self.walls = walls#[(wall_start_meters, wall_end_meters)]
        #self.wall_start_meters = wall_start_meters
        #self.wall_end_meters = wall_end_meters
        self.drone_location_meters = drone_location_meters
        """self.drone_yaw_degrees = drone_yaw_degrees"""
        self.scale_factor = 50  # Scale factor to convert meters units to pixels
        self.lidar_noise_meters_standard_dev = lidar_noise_meters_standard_dev

        self.lidar_angle_step_degrees = 1

        self.title('Drone Lidar')
        self.geometry('800x600')
        self.create_figure()
        self.update_canvas()

    def draw_drone(self):
        """
        Draw the drone on the matplotlib figure.
        """
        drone_center = (self.drone_location_meters[0] * self.scale_factor, self.drone_location_meters[1] * self.scale_factor)  # Fixed position at the center of the screen
        drone_yaw_rad = 0 #math.radians(self.drone_yaw_degrees)

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
            wall_start_meters, wall_end_meters = wall
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

    def draw_point(self, point_meters):
        """
        Draw a point on the matplotlib figure based on its meters coordinates.

        Args:
            point_meters (tuple): The meters coordinates of the point.
        """
        point_x = point_meters[0] * self.scale_factor
        point_y = point_meters[1] * self.scale_factor
        self.ax.plot(point_x, point_y, 'ko', markersize=1)
    
    def draw_lidar_points(self):
        """
        Draw LIDAR points on the matplotlib figure.
        """
        for lidar_angle, lidar_distance in self.lidar_readings_angle_deg_dist_m:
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_readings_xy_meters_relative = lidar_reading_to_deltaxy(lidar_angle, lidar_distance)
                lidar_readings_xy_meters_absolute = tuple(a + b for a, b in zip(lidar_readings_xy_meters_relative, self.drone_location_meters))
                self.draw_point(lidar_readings_xy_meters_absolute)

    def get_lidar_readings_angle_deg_dist_m(self):
        """
        Returns an array of tuples containing (angle, distance) values for LIDAR readings.

        The function iterates through angles from 0 to 360 degrees, stepping by self.lidar_angle_step_degrees.
        If the LIDAR doesn't hit a wall, the distance value in the tuple is set to None.

        Angles start at 0 degrees North and move clockwise. 90 degrees is right, and 270 degrees is left relative to the drone.

        Returns:
            lidar_readings (list of tuples): A list of tuples where each tuple contains the angle (float) and the LIDAR distance (float or None).
        """
        lidar_readings = []
        angle = 0

        while angle < 360:
            angle_rad = math.radians(( - angle + 360 + 90) % 360)

            dx = math.cos(angle_rad)
            dy = math.sin(angle_rad)

            distance_min = None

            for wall in self.walls:
                wall_start_meters, wall_end_meters = wall

                t_denominator = (wall_end_meters[1] - wall_start_meters[1]) * dx - (wall_end_meters[0] - wall_start_meters[0]) * dy

                if t_denominator == 0:
                    angle += self.lidar_angle_step_degrees
                    continue

                t_numerator = (wall_start_meters[0] - self.drone_location_meters[0]) * dy - (wall_start_meters[1] - self.drone_location_meters[1]) * dx
                t = t_numerator / t_denominator

                if t < 0 or t > 1:
                    angle += self.lidar_angle_step_degrees
                    continue

                u_numerator = (wall_start_meters[0] - self.drone_location_meters[0]) * (wall_end_meters[1] - wall_start_meters[1]) - (wall_start_meters[1] - self.drone_location_meters[1]) * (wall_end_meters[0] - wall_start_meters[0])
                u = u_numerator / t_denominator

                if u < 0:
                    angle += self.lidar_angle_step_degrees
                    continue

                # The random component is from a standard normal distribution
                # The mean distance is the lidar reading and the standard deviation is lidar_noise_meters_standard_dev
                distance = (u + self.lidar_noise_meters_standard_dev * np.random.randn(1))[0]

                if ((distance_min == None) or (distance < distance_min)):
                    distance_min = distance
                    
            lidar_readings.append((angle, distance_min))

            angle += self.lidar_angle_step_degrees

        self.lidar_readings_angle_deg_dist_m = lidar_readings

        # Initialize the array
        self.lidar_readings_xy_meters_absolute = []

        for lidar_angle, lidar_distance in self.lidar_readings_angle_deg_dist_m:
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_reading_xy_meters_relative = lidar_reading_to_deltaxy(lidar_angle, lidar_distance)
                lidar_reading_xy_meters_absolute = tuple(a + b for a, b in zip(lidar_reading_xy_meters_relative, self.drone_location_meters))

                # Add the point to the array
                self.lidar_readings_xy_meters_absolute.append(lidar_reading_xy_meters_absolute)

        return lidar_readings

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

    def create_figure(self):
        """
        Create a matplotlib figure to display the drone, wall, and LIDAR data.
        """
        # Create a figure
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax = self.fig.add_subplot(1, 1, 1)

        # Set the x and y axis limits
        drone_pixel_location_x = self.drone_location_meters[0] * self.scale_factor
        drone_pixel_location_y = self.drone_location_meters[1] * self.scale_factor

        # The x and y limits here keep everything relative to the position of the drone!
        self.ax.set_xlim(drone_pixel_location_x - 400,
                        drone_pixel_location_x + 400)
        self.ax.set_ylim(drone_pixel_location_y - 300,
                        drone_pixel_location_y + 300)
        
        self.x_window_min = self.drone_location_meters[0] - 400 / self.scale_factor
        self.x_window_max = self.drone_location_meters[0] + 400 / self.scale_factor

        # Set the x and y axis ticks
        x_ticks = np.arange(drone_pixel_location_x - 400, drone_pixel_location_x + 400, 100)
        y_ticks = np.arange(drone_pixel_location_y - 300, drone_pixel_location_y + 300, 100)
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)

        # Set the x and y axis tick labels in meters units
        x_tick_labels = np.round((x_ticks - drone_pixel_location_x) / self.scale_factor + self.drone_location_meters[0], 1)
        y_tick_labels = np.round((y_ticks - drone_pixel_location_y) / self.scale_factor + self.drone_location_meters[1], 1)
        self.ax.set_xticklabels(x_tick_labels)
        self.ax.set_yticklabels(y_tick_labels)