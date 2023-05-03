"""
Was thinking about splitting up Lidar_and_Wall_Simulator_With_GUI into this GUI file
"""

import tkinter as tk
import math
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from matplotlib.figure import Figure


class GUI(tk.Tk):
    def __init__(self, scale_factor=25):
        tk.Tk.__init__(self)
        self.scale_factor = (
            scale_factor  # Scale factor to convert meters units to pixels
        )
        self.title("Drone Lidar")
        self.geometry("800x600")
        self.create_figure()
        self.update_canvas()

    def draw_walls(self, walls):
        """
        Draw the wall on the matplotlib figure.
        """

        for wall in walls:
            wall_start_meters = wall.wall_start_point_relative_m
            wall_end_meters = wall.wall_end_point_relative_m
            # wall_start_meters, wall_end_meters = wall
            self.draw_wall_from_coordinates(wall_start_meters, wall_end_meters)

    def draw_lidar_points(self, lidar_readings):
        """
        Draw LIDAR points on the matplotlib figure.
        """
        for lidar_reading in lidar_readings:
            # If lidar_distance != null
            if lidar_reading.total_relative_distance_m is not None:
                self.draw_point(
                    lidar_reading.x_relative_distance_m,
                    lidar_reading.y_relative_distance_m,
                )

    def draw_drone(self):
        """
        Draw the drone on the matplotlib figure.
        """
        drone_center = (0, 0)  # Fixed position at the center of the screen
        drone_yaw_rad = 0

        triangle_points = [
            (
                drone_center[0] + 10 * math.sin(drone_yaw_rad),
                drone_center[1] + 10 * math.cos(drone_yaw_rad),
            ),
            (
                drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(150)),
                drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(150)),
            ),
            (
                drone_center[0] + 5 * math.sin(drone_yaw_rad + math.radians(-150)),
                drone_center[1] + 5 * math.cos(drone_yaw_rad + math.radians(-150)),
            ),
        ]

        drone_shape = plt.Polygon(triangle_points, edgecolor="blue", fill=True)
        self.ax.add_patch(drone_shape)

    def add_text(self, text):
        self.ax.text(
            -390, 290, text, fontsize=12, color="black", verticalalignment="top"
        )

    def draw_wall_from_coordinates(
        self, wall_start_meters_tuple, wall_end_meters_tuple, color="r-"
    ):
        """
        Draw the wall on the matplotlib figure.
        """
        wall_start = (
            wall_start_meters_tuple[0] * self.scale_factor,
            wall_start_meters_tuple[1] * self.scale_factor,
        )
        wall_end = (
            wall_end_meters_tuple[0] * self.scale_factor,
            wall_end_meters_tuple[1] * self.scale_factor,
        )
        self.ax.plot([wall_start[0], wall_end[0]], [wall_start[1], wall_end[1]], color)

    def draw_point(self, point_x_m, point_y_m):
        """
        Draw a point on the matplotlib figure based on its meters coordinates.

        Args:
            point_meters (tuple): The meters coordinates of the point.
        """
        point_x = point_x_m * self.scale_factor
        point_y = point_y_m * self.scale_factor
        self.ax.plot(point_x, point_y, "ko", markersize=1)

    def update_canvas(self):
        """
        Update the canvas to display the latest state of the matplotlib figure.
        """
        # Create a canvas to display the figure (or update the existing canvas)
        if not hasattr(self, "canvas"):
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
        drone_pixel_location_x = 0
        drone_pixel_location_y = 0

        # The x and y limits here keep everything relative to the position of the drone!
        self.ax.set_xlim(drone_pixel_location_x - 400, drone_pixel_location_x + 400)
        self.ax.set_ylim(drone_pixel_location_y - 300, drone_pixel_location_y + 300)

        self.x_window_min = 0 - 400 / self.scale_factor
        self.x_window_max = 0 + 400 / self.scale_factor

        # Set the x and y axis ticks
        x_ticks = np.arange(
            drone_pixel_location_x - 400, drone_pixel_location_x + 400, 100
        )
        y_ticks = np.arange(
            drone_pixel_location_y - 300, drone_pixel_location_y + 300, 100
        )
        self.ax.set_xticks(x_ticks)
        self.ax.set_yticks(y_ticks)

        # Set the x and y axis tick labels in meters units
        x_tick_labels = np.round(
            (x_ticks - drone_pixel_location_x) / self.scale_factor, 1
        )
        y_tick_labels = np.round(
            (y_ticks - drone_pixel_location_y) / self.scale_factor, 1
        )
        self.ax.set_xticklabels(x_tick_labels)
        self.ax.set_yticklabels(y_tick_labels)
