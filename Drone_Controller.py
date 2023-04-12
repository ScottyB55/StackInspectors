"""
Chat GPT Ideas with some Editing

An effective approach for a drone to follow the closest wall for inspecting smokestacks would be to combine lidar data with 
a wall-following algorithm. Here's an outline of the steps involved:
    1. Preprocess lidar data: Filter and cluster the lidar data to identify the nearest wall points. 
    You can use the DBSCAN clustering method mentioned in the previous answer or any other suitable clustering technique.

    2. Estimate the wall's orientation: Perform linear regression on the nearest wall points to estimate the wall's orientation. 
    This will give you a line of best fit representing the wall's orientation. 
    Alternatively, you can also use techniques like RANSAC for a more robust estimation in the presence of noise.

    3. Calculate the drone's distance from the wall's line of best fit. Take this distance, and feed it back into the drone's
    orientation rotation (attitude) parallel to the wall's line of best fit. The user's mouse input should control the drone's
    orientation rotation (attitude) perpendicular to the wall line of best fit to allow the drone to move up and down the wall.

    4. Control the drone's position and orientation: Implement a PID controller or another suitable control algorithm 
    to steer the drone towards the target position while maintaining the desired distance from the wall. 
    The controller should handle both the drone's position and orientation to ensure it remains parallel to the wall.

    5. Monitor and update: Continuously update the drone's position, orientation, and lidar data. 
    If the drone approaches a corner or an obstacle, adjust its trajectory accordingly. 
    You can use additional sensors, such as cameras or ultrasonic sensors, to enhance the drone's perception of its surroundings.

    6. Safety measures: Implement safety measures like collision avoidance and failsafe mechanisms to ensure the drone's 
    safe operation in the smokestack environment.

This approach should allow the drone to follow the closest wall while inspecting smokestacks. 
You can further improve the performance by fusing data from multiple sensors and implementing advanced control algorithms.

Another thing to add: we want two controllers
1. One that makes the drone face the wall
2. One that makes 

Terminal to send simple messages over terminal
Start scan, stop scan
Turn on lindar, turn off lidar, ask for kinds of messages
The encoding messages from the lidar sensor
Nothing is readable

Firmware update
"""

from Drone_Class import Simulated_Drone_Realistic_Physics, Simulated_Drone_Simple_Physics
import time
import threading
from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized

from Drone_Class import Drone

import numpy as np
import matplotlib.pyplot as plt
from scipy import odr
import math

# Define the linear function for ODR
def linear_function(params, x):
    m, c = params
    return m * x + c

def point_line_distance(x0, y0, m, b):
    return abs(m * x0 - y0 + b) / math.sqrt(m**2 + 1)

def distance(point1, point2):
    """Calculate the distance between two points."""
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def dot_product(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def scalar_multiply(v, scalar):
    return (v[0] * scalar, v[1] * scalar)

def vector_subtract(v1, v2):
    return (v1[0] - v2[0], v1[1] - v2[1])

mouse_position_normalized_to_meters_velocity = 1

class Drone_Controller:
    def __init__(self, Drone):
        self.Drone = Drone
        self.target_distance = 2   # the target distance between the drone and the wall
        self.target_angle = 90      # the target angle between the drone
        self.velocity_x_setpoint = 0
        self.velocity_y_setpoint = 0
        self.distance_error = None
    
    def update_drone_velocity(self):
        # Have calculations to determine the drone velocity here
        drone_location_meters = self.Drone.drone_location_meters

        closest_point = self.find_closest_point()

        # Find the displacement between the drone and the closest point
        delta_x = closest_point[0] - drone_location_meters[0]
        delta_y = closest_point[1] - drone_location_meters[1]

        distance = math.sqrt(delta_x**2 + delta_y**2)

        # Scale this distance to a unit vector
        delta_x_unit = delta_x / distance
        delta_y_unit = delta_y / distance

        derivative_error = 0
        distance_error_prev = self.distance_error
        self.distance_error = distance - self.target_distance

        if (distance_error_prev != None):
            derivative_error = self.distance_error - distance_error_prev
        
        Kp = 1
        Kd = 5

        # put in the PID setpoint that is in line with the displacement to the closest point
        self.velocity_x_setpoint = delta_x_unit * (Kp * self.distance_error + Kd * derivative_error)
        self.velocity_y_setpoint = delta_y_unit * (Kp * self.distance_error + Kd * derivative_error)

        # Get the mouse input so we can move the drone parallel to the line between nearest point & drone
        mouse_x, mouse_y = mouse_relative_position_from_center_normalized()

        # Calculate the projection of (mouse_x, mouse_y) onto (delta_x_unit, delta_y_unit)
        projection_scale = dot_product((mouse_x, mouse_y), (delta_x_unit, delta_y_unit))
        projection = scalar_multiply((delta_x_unit, delta_y_unit), projection_scale)

        # Subtract the projection from the original vector to get the perpendicular component
        perpendicular_component = vector_subtract((mouse_x, mouse_y), projection)

        # Add the mouse component onto the setpoint (perpendicular to the dist between closest point & drone)
        self.velocity_x_setpoint += perpendicular_component[0]
        self.velocity_y_setpoint += perpendicular_component[1]

        print(f"distance = {distance} distance_error = {self.distance_error} velocity_setpoint = {self.velocity_x_setpoint}, {self.velocity_y_setpoint}")

        # Update the drone's velocity using defaults for yaw and throttle
        self.Drone.set_attitude_setpoint(self.velocity_x_setpoint, self.velocity_y_setpoint)

        # drone_app.set_attitude_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))

    def find_closest_point(self):
        lidar_readings = self.Drone.lidar_and_wall_sim_with_gui.lidar_readings_xy_meters_absolute

        drone_location_meters = self.Drone.drone_location_meters

        if (True):
            min_distance = None
            closest_point = None

            for point in lidar_readings:
                dist = distance(drone_location_meters, point)
                if min_distance is None or dist < min_distance:
                    min_distance = dist
                    closest_point = point
            return (closest_point)
        else:
            # Line implementation
            # Separate the x and y coordinates
            x_values = [point[0] for point in lidar_readings]
            y_values = [point[1] for point in lidar_readings]

            # Option 1: ODR regression (works for both horizontal and vertical)
            # Create a model for the orthogonal distance regression
            linear_model = odr.Model(linear_function)
            # Create the input data for the ODR
            data = odr.RealData(x_values, y_values)
            # Initialize the ODR with the model, the data, and an initial guess for the parameters (m and c)
            odr_instance = odr.ODR(data, linear_model, beta0=[1, 0])
            # Run the ODR
            output = odr_instance.run()
            # Extract the fitted parameters (m and c)
            slope, intercept = output.beta

            # Option 2: linear regression (not as good for vertical)
            # slope, intercept = np.polyfit(x_values, y_values, 1)

            # distance = point_line_distance(self.Drone.drone_location_meters[0], self.Drone.drone_location_meters[1], 
            #                                slope, intercept)

            drone_location_meters = self.Drone.drone_location_meters

            # Find the point on the line that is closest to the given point
            x_on_line = (drone_location_meters[0] + slope * drone_location_meters[1] - slope * intercept) / (1 + slope**2)
            y_on_line = slope * x_on_line + intercept

            # Plot the perceived line

            x0 = self.Drone.lidar_and_wall_sim_with_gui.x_window_min
            x1 = self.Drone.lidar_and_wall_sim_with_gui.x_window_max

            y0 = slope * x0 + intercept
            y1 = slope * x1 + intercept

            self.Drone.lidar_and_wall_sim_with_gui.draw_wall_from_coordinates((x0, y0), (x1, y1), 'b-')
        
            return (x_on_line, y_on_line)

    def draw_perceived_wall(self):
        pass

    def run(self):
        # Get the lidar data
        self.Drone.lidar_and_wall_sim_with_gui.lidar_readings_xy_meters_absolute

        # Optional: factor the drone's roll & pitch into the lidar data

        # Optional: cluster on the lidar data

        # TODO: get the line of best fit for the lidar data

        # Control Loop 1: adjust the drone's target_yaw to make the drone face towards the wall
        #   1. Calculate the angle between the line where the drone is facing & the lidar line of best fit
        #   2. This angle should be 90 degrees. Pass the error difference through a control function & feed back to the drone's target_yaw

        # Control Loop 2: adjust the drone's roll & pitch to make the drone maintain a constant distance from the wall
        #   1. Calculate the closest distance between the drone and the lidar line of best fit
        #   2. Establish some sort of matrix relationship as follows:
        #       Input: target angles parallel & perpendicular to the lidar line of best fit
        #       Output: target_roll and target_pitch for the drone
        #   3. Set the parallel input component to the difference between the desired distance & the actual distance
        #   4. Set the perpendicular input component to the mouse x coordinates

        # Additional Control: remember that there is an additional hover_thrust input that controls how much the drone moves up or down
        #   Set this proportional to the mouse y coordinates

        # Use something like the set_attitude() function for self.Drone.set_drone_velocity()

def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    drone_controller.Drone.update_lidar_readings()
    
    while True:
        drone_controller.update_drone_velocity()

        # drone_app.set_attitude_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))
        time.sleep(timestep)

        drone_controller.Drone.update_location_meters(timestep)
        drone_controller.Drone.update_lidar_readings()
        drone_controller.Drone.wipe_gui()
        # drone_controller.draw_perceived_wall()
        drone_controller.Drone.update_gui()

        


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    walls = [((1, -2), (1, 4)), ((1, -2), (7, -2))]

    #wall_start_meters = (-2, 0)
    #wall_end_meters = (2, 0)

    # Define the initial meters coordinates of the drone
    drone_location_meters = (1.5, 1)
    drone_location_meters = (4, -1.5)

    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Simulated_Drone_Simple_Physics(walls, drone_location_meters, lidar_noise_meters_standard_dev)
    #drone_app = Simulated_Drone_Realistic_Physics(walls, drone_location_meters, lidar_noise_meters_standard_dev)

    drone_controller = Drone_Controller(drone_app)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_controller.Drone,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_controller.Drone.lidar_and_wall_sim_with_gui.mainloop()