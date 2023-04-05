from mouse_and_keyboard import mouse_relative_position_from_center_normalized

import tkinter as tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import matplotlib.pyplot as plt

import numpy as np

import time
import threading

from collections import namedtuple

#
LidarReading = namedtuple('LidarReading', ['angle_degrees', 'gps_distance'])

"""
Definitions
0 degrees is top
90 degrees is right
180 degrees is bottom
270 degrees is left
"""
class DroneLidar(tk.Tk):
    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, drone_yaw_degrees):
        super().__init__()

        self.wall_start_gps = wall_start_gps
        self.wall_end_gps = wall_end_gps
        self.drone_gps = drone_gps
        self.drone_yaw_degrees = drone_yaw_degrees
        self.scale_factor = 200  # Scale factor to convert GPS units to pixels

        self.title('Drone Lidar')
        self.geometry('800x600')
        self.create_figure()
        self.update_canvas()

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

        """
        # Draw the drone as an elongated triangle
        self.draw_drone(ax)

        # Draw the wall line
        self.draw_wall(ax)
        """
        
    def update_canvas(self):
        # Create a canvas to display the figure (or update the existing canvas)
        if not hasattr(self, 'canvas'):
            self.canvas = FigureCanvasTkAgg(self.fig, master=self)
            self.canvas.get_tk_widget().pack()
        else:
            self.canvas.figure = self.fig
            self.canvas.draw()

    def draw_drone(self):
        drone_center = (self.drone_gps[0] * self.scale_factor, self.drone_gps[1] * self.scale_factor)  # Fixed position at the center of the screen
        drone_yaw_rad = math.radians(self.drone_yaw_degrees)

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

    def update_drone_gps(self, new_gps):
        self.drone_gps = new_gps

    def update_drone_yaw_degrees(self, new_yaw):
        self.drone_yaw_degrees = new_yaw

    def get_lidar_readings_gps(self):
        lidar_readings_gps = [None] * 360

        for i in range(360):
            angle = (self.drone_yaw_degrees - i + 90) % 360
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

def lidar_reading_to_deltaxy(lidar_angle, distance):
    shifted_angle = 90 - lidar_angle
    shifted_angle_rad = math.radians(shifted_angle)

    delta_x = math.cos(shifted_angle_rad) * distance
    delta_y = math.sin(shifted_angle_rad) * distance

    return delta_x, delta_y

def run_simulation(app):
    while True:
        time.sleep(1)
        mouse_x, mouse_y = mouse_relative_position_from_center_normalized()
        new_gps = (app.drone_gps[0] + mouse_x * 0.2, app.drone_gps[1] - mouse_y * 0.2)
        app.update_drone_gps(new_gps)
        app.create_figure()
        app.draw_drone()
        app.draw_wall()

        lidar_readings_gps = app.get_lidar_readings_gps()
        print(lidar_readings_gps)

        # For each lidar_distance reading in lidar_readings_gps,
        for index, lidar_distance in enumerate(lidar_readings_gps):
            # If lidar_distance != null
            if lidar_distance is not None:
                lidar_gps = lidar_reading_to_deltaxy(index, lidar_distance)
                # call this function: app.draw_point(lidar_gps)
                # app.draw_point(lidar_gps - drone_gps)
                # app.draw_point(wall_start_gps)
                # app.draw_point(wall_end_gps)
                app.draw_point(tuple(a + b for a, b in zip(lidar_gps, drone_gps)))

                # app.draw_point(tuple(-b for a, b in zip(lidar_gps, drone_gps)));
                # app.draw_point(tuple(- b - a for a, b in zip(lidar_gps, drone_gps)));
        
        app.update_canvas()
        

if __name__ == '__main__':
    wall_start_gps = (0, -0.5)
    wall_end_gps = (0, 0.5)
    drone_gps = (0.5, 0.5)
    drone_yaw_degrees = 0

    app = DroneLidar(wall_start_gps, wall_end_gps, drone_gps, drone_yaw_degrees)

    # Start a new thread to move the drone left every second
    move_drone_thread = threading.Thread(target=run_simulation, args=(app,))
    move_drone_thread.daemon = True
    move_drone_thread.start()

    app.mainloop()