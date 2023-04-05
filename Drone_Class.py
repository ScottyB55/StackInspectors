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
    shifted_angle = 90 - lidar_angle
    shifted_angle_rad = math.radians(shifted_angle)

    delta_x = math.cos(shifted_angle_rad) * distance
    delta_y = math.sin(shifted_angle_rad) * distance

    return delta_x, delta_y



class Drone:
    def __init__(self):
        pass

    def get_lidar_readings_gps(self):
        pass

    def set_drone_velocity(self, drone_velocity):
        pass

    def take_off(self):
        # Implement take_off logic
        pass

    def land(self):
        # Implement land logic
        pass

    # Add other common methods and properties for all drones


class Real_Life_Drone(Drone):
    def __init__(self):
        super().__init__()


class Simulated_Drone(Drone, tk.Tk):

    def __init__(self, wall_start_gps, wall_end_gps, drone_gps): #, drone_yaw_degrees
        Drone.__init__(self)
        tk.Tk.__init__(self)

        self.wall_start_gps = wall_start_gps
        self.wall_end_gps = wall_end_gps
        self.drone_gps = drone_gps
        """self.drone_yaw_degrees = drone_yaw_degrees"""
        self.scale_factor = 200  # Scale factor to convert GPS units to pixels

        self.title('Drone Lidar')
        self.geometry('800x600')
        self.create_figure()
        self.update_canvas()

    def draw_drone(self):
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
        wall_start = (self.wall_start_gps[0] * self.scale_factor,
                      self.wall_start_gps[1] * self.scale_factor)
        wall_end = (self.wall_end_gps[0] * self.scale_factor,
                    self.wall_end_gps[1] * self.scale_factor)
        self.ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], 'r-')

    def draw_point(self, point_gps):
        point_x = point_gps[0] * self.scale_factor
        point_y = point_gps[1] * self.scale_factor
        self.ax.plot(point_x, point_y, 'ko', markersize=1)
    
    def draw_lidar_points(self):
        for index, lidar_distance in enumerate(self.get_lidar_readings_gps()):
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_gps = lidar_reading_to_deltaxy(index, lidar_distance)
                # call this function: drone_app.draw_point(lidar_gps)
                # drone_app.draw_point(lidar_gps - drone_gps)
                # drone_app.draw_point(wall_start_gps)
                # drone_app.draw_point(wall_end_gps)
                self.draw_point(tuple(a + b for a, b in zip(lidar_gps, self.drone_gps)))

                # drone_app.draw_point(tuple(-b for a, b in zip(lidar_gps, drone_gps)));
                # drone_app.draw_point(tuple(- b - a for a, b in zip(lidar_gps, drone_gps)));

    def get_lidar_readings_gps(self):
        lidar_readings_gps = [None] * 360

        for i in range(360):
            angle = ( - i + 90) % 360 # Assuming this is 0. TODO: implement. self.drone_yaw_degrees
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

    def update_canvas(self):
        # Create a canvas to display the figure (or update the existing canvas)
        if not hasattr(self, 'canvas'):
            self.canvas = FigureCanvasTkAgg(self.fig, master=self)
            self.canvas.get_tk_widget().pack()
        else:
            self.canvas.figure = self.fig
            self.canvas.draw()

    def create_figure(self):
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
        self.drone_velocity = drone_velocity


class Simulated_Drone_Simple_Physics(Simulated_Drone):
    def __init__(self, wall_start_gps, wall_end_gps, drone_gps): #, drone_yaw_degrees
        super().__init__(wall_start_gps, wall_end_gps, drone_gps)
    
    def update_physics(self, timestep):
        # new_gps = (drone_app.drone_gps[0] + mouse_x * 0.2, drone_app.drone_gps[1] - mouse_y * 0.2)
        self.drone_gps = tuple(a + b * timestep for a, b in zip(self.drone_gps, self.drone_velocity))

def run_simulation(drone_app):
    timestep = 1
    mouse_position_normalized_to_gps_velocity = 0.2
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
    def __init__(self):
        super().__init__()


if __name__ == '__main__':
    wall_start_gps = (0, -0.5)
    wall_end_gps = (0, 0.5)
    drone_gps = (0.5, 0.5)
    drone_yaw_degrees = 0

    drone_app = Simulated_Drone_Simple_Physics(wall_start_gps, wall_end_gps, drone_gps) #, drone_yaw_degrees)

    # Start a new thread to move the drone left every second
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    move_drone_thread.daemon = True
    move_drone_thread.start()

    drone_app.mainloop()