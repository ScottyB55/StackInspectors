# This is based on V2_Lidar_Sim_Class.py in the Reference Files Folder

from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized

import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import matplotlib.pyplot as plt

import numpy as np

import time
import threading

from collections import namedtuple

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



class Drone:
    """
    Base class for representing a drone with common methods and properties for all drones.
    """
    def __init__(self):
        """
        Initialize a Drone object.
        """
        pass

    def get_lidar_readings_gps(self):
        """
        Get LIDAR readings for the drone in GPS coordinates.
        """
        pass

    def set_drone_velocity(self, drone_velocity):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        pass

    def take_off(self):
        """
        Implement the take_off logic for the drone.
        """
        pass

    def land(self):
        """
        Implement the land logic for the drone.
        """
        pass

    # Add other common methods and properties for all drones


class Real_Life_Drone(Drone):
    """
    Represents a real-life drone inheriting from the base Drone class.
    """
    def __init__(self):
        """
        Initialize a Real_Life_Drone object.
        """
        super().__init__()


class Simulated_Drone(Drone, tk.Tk):
    """
    Represents a simulated drone with a GUI for displaying LIDAR data.

    Args:
        wall_start_gps (tuple): The GPS coordinates of the starting point of the wall.
        wall_end_gps (tuple): The GPS coordinates of the ending point of the wall.
        drone_gps (tuple): The GPS coordinates of the drone.
        lidar_noise_gps_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev): #, drone_yaw_degrees
        Drone.__init__(self)
        tk.Tk.__init__(self)

        self.wall_start_gps = wall_start_gps
        self.wall_end_gps = wall_end_gps
        self.drone_gps = drone_gps
        """self.drone_yaw_degrees = drone_yaw_degrees"""
        self.scale_factor = 200  # Scale factor to convert GPS units to pixels
        self.lidar_noise_gps_standard_dev = lidar_noise_gps_standard_dev

        self.lidar_angle_step_degrees = 1

        self.title('Drone Lidar')
        self.geometry('800x600')
        self.create_figure()
        self.update_canvas()

    def draw_drone(self):
        """
        Draw the drone on the matplotlib figure.
        """
        drone_center = (self.drone_gps[0] * self.scale_factor, self.drone_gps[1] * self.scale_factor)  # Fixed position at the center of the screen
        drone_yaw_rad = 0 #math.radians(self.drone_yaw_degrees)

        triangle_points = [
            (drone_center[0] + 10 * math.sin(drone_yaw_rad), drone_center[1] + 10 * math.cos(drone_yaw_rad)),
            (drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(150)), drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(150))),
            (drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(-150)), drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(-150)))
        ]

        drone_shape = plt.Polygon(triangle_points, edgecolor='blue', fill=True)
        self.ax.add_patch(drone_shape)

    def draw_wall(self):
        """
        Draw the wall on the matplotlib figure.
        """
        wall_start = (self.wall_start_gps[0] * self.scale_factor,
                      self.wall_start_gps[1] * self.scale_factor)
        wall_end = (self.wall_end_gps[0] * self.scale_factor,
                    self.wall_end_gps[1] * self.scale_factor)
        self.ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], 'r-')

    def draw_point(self, point_gps):
        """
        Draw a point on the matplotlib figure based on its GPS coordinates.

        Args:
            point_gps (tuple): The GPS coordinates of the point.
        """
        point_x = point_gps[0] * self.scale_factor
        point_y = point_gps[1] * self.scale_factor
        self.ax.plot(point_x, point_y, 'ko', markersize=1)
    
    def draw_lidar_points(self):
        """
        Draw LIDAR points on the matplotlib figure.
        """
        for lidar_angle, lidar_distance in self.get_lidar_readings_gps():
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_gps = lidar_reading_to_deltaxy(lidar_angle, lidar_distance)
                self.draw_point(tuple(a + b for a, b in zip(lidar_gps, self.drone_gps)))

    def get_lidar_readings_gps(self):
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

            t_denominator = (self.wall_end_gps[1] - self.wall_start_gps[1]) * dx - (self.wall_end_gps[0] - self.wall_start_gps[0]) * dy

            if t_denominator == 0:
                angle += self.lidar_angle_step_degrees
                continue

            t_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * dy - (self.wall_start_gps[1] - self.drone_gps[1]) * dx
            t = t_numerator / t_denominator

            if t < 0 or t > 1:
                angle += self.lidar_angle_step_degrees
                continue

            u_numerator = (self.wall_start_gps[0] - self.drone_gps[0]) * (self.wall_end_gps[1] - self.wall_start_gps[1]) - (self.wall_start_gps[1] - self.drone_gps[1]) * (self.wall_end_gps[0] - self.wall_start_gps[0])
            u = u_numerator / t_denominator

            if u < 0:
                angle += self.lidar_angle_step_degrees
                continue

            # The random component is from a standard normal distribution
            # The mean distance is the lidar reading and the standard deviation is lidar_noise_gps_standard_dev
            distance = u + self.lidar_noise_gps_standard_dev * np.random.randn(1)
            lidar_readings.append((angle, distance[0]))

            angle += self.lidar_angle_step_degrees

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
        drone_pixel_location_x = self.drone_gps[0] * self.scale_factor
        drone_pixel_location_y = self.drone_gps[1] * self.scale_factor

        # The x and y limits here keep everything relative to the position of the drone!
        self.ax.set_xlim(drone_pixel_location_x - 400,
                        drone_pixel_location_x + 400)
        self.ax.set_ylim(drone_pixel_location_y - 300,
                        drone_pixel_location_y + 300)

        # Set the x and y axis tick labels in GPS units
        self.ax.set_xticks(np.arange(drone_pixel_location_x - 400, drone_pixel_location_x + 400, 100))
        self.ax.set_yticks(np.arange(drone_pixel_location_y - 300, drone_pixel_location_y + 300, 100))
        self.ax.set_xticklabels(np.round(np.arange(self.drone_gps[0] - 4, self.drone_gps[0] + 4, 1), 1))
        self.ax.set_yticklabels(np.round(np.arange(self.drone_gps[1] - 3, self.drone_gps[1] + 3, 1), 1))

    def set_drone_velocity(self, drone_velocity):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        self.drone_velocity = drone_velocity


class Simulated_Drone_Simple_Physics(Simulated_Drone):
    """
    Represents a simulated drone with simple physics.

    Args:
        wall_start_gps (tuple): The GPS coordinates of the starting point of the wall.
        wall_end_gps (tuple): The GPS coordinates of the ending point of the wall.
        drone_gps (tuple): The GPS coordinates of the drone.
        lidar_noise_gps_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev): #, drone_yaw_degrees
        super().__init__(wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev)
    
    def update_physics(self, timestep):
        """
        Update the physics of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's physics.
        """
        self.drone_gps = tuple(a + b * timestep for a, b in zip(self.drone_gps, self.drone_velocity))

def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_gps_velocity = 0.4
    while True:
        drone_app.create_figure()
        drone_app.draw_drone()
        drone_app.draw_wall()
        drone_app.draw_lidar_points()
        drone_app.update_canvas()
        
        drone_app.set_drone_velocity(tuple(x * mouse_position_normalized_to_gps_velocity for x in mouse_relative_position_from_center_normalized()))

        print("loop" + str(drone_app.drone_gps))
        time.sleep(timestep)
        drone_app.update_physics(timestep)


class Simulated_Drone_QGroundControl_Physics(Simulated_Drone):
    """
    Represents a simulated drone with QGroundControl physics.
    """

    def __init__(self):
        """
        Initialize a Simulated_Drone_QGroundControl_Physics object.
        """
        super().__init__()


if __name__ == '__main__':
    # Define the starting and ending GPS coordinates of the wall
    wall_start_gps = (0, -1)
    wall_end_gps = (0, 1)
    # Define the initial GPS coordinates of the drone
    drone_gps = (0.5, 0.5)
    # Define the standard deviation of the LIDAR noise in GPS units
    lidar_noise_gps_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Simulated_Drone_Simple_Physics(wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_app.mainloop()