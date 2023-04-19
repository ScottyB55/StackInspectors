from Drone_Class import Drone, Simulated_Drone_Realistic_Physics, Simulated_Drone_Simple_Physics, Real_Drone_Realistic_Physics
from Lidar_and_Wall_Simulator_With_GUI import Wall, LidarReading, Lidar_and_Wall_Simulator_With_GUI
import time
import threading
#from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized

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
        self.target_distance = 0.5   # the target distance between the drone and the wall
        self.target_angle = 90      # the target angle between the drone
        self.velocity_x_setpoint = 0
        self.velocity_y_setpoint = 0
        self.distance_error = None
    
    def update_drone_velocity(self):

        closest_point = self.find_closest_point()

        # Find the displacement between the drone and the closest point
        delta_x = closest_point.x_relative_distance_m
        delta_y = closest_point.y_relative_distance_m

        distance = closest_point.total_relative_distance_m

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

        # Get the mouse input so we can move the drone perpendicular to the line between nearest point & drone
        mouse_x, mouse_y = 0, 0#mouse_relative_position_from_center_normalized()

        # Calculate the projection of (mouse_x, mouse_y) onto (delta_x_unit, delta_y_unit)
        projection_scale = dot_product((mouse_x, mouse_y), (delta_x_unit, delta_y_unit))
        projection = scalar_multiply((delta_x_unit, delta_y_unit), projection_scale)

        # Subtract the projection from the original vector to get the perpendicular component
        perpendicular_component = vector_subtract((mouse_x, mouse_y), projection)

        # Add the mouse component onto the setpoint (perpendicular to the dist between closest point & drone)
        self.velocity_x_setpoint += perpendicular_component[0]
        self.velocity_y_setpoint += perpendicular_component[1]

        #print(f"distance = {distance} distance_error = {self.distance_error} velocity_setpoint = {self.velocity_x_setpoint}, {self.velocity_y_setpoint}")

        K_YAW_CTRL = 50

        current_yaw = self.Drone.get_current_yaw_angle()
        target_yaw = current_yaw + closest_point.lidar_angle_degrees
        error_yaw = target_yaw - current_yaw
        while (abs(error_yaw) > 180):
            if error_yaw > 0:
                error_yaw -= 360
            else:
                error_yaw += 360

        error_yaw = error_yaw * K_YAW_CTRL / 100

        setpoint_yaw1 = (current_yaw + error_yaw + 360) % 360

        setpoint_yaw2 = (self.Drone.get_current_yaw_angle() + closest_point.lidar_angle_degrees + 360) % 360

        #print(f"Pt1: {setpoint_yaw1} Pt2: {setpoint_yaw2}")

        # Hover thrust ranges from 0 to 1
        # Mouse_Y ranges from -1 to 1
        hover_thrust_range_fraction = 0.5
        hover_thrust_setpoint = 0.5 + mouse_y * hover_thrust_range_fraction / 2
        # TODO: add a feature to allow the user to fix the hover thrust at a particular level

        # Update the drone's velocity using defaults for yaw and throttle
        self.Drone.set_attitude_setpoint(self.velocity_x_setpoint, self.velocity_y_setpoint, setpoint_yaw1, hover_thrust_setpoint)

        self.closest_point = closest_point
        self.error_yaw = error_yaw

        # drone_app.set_attitude_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))

    def find_closest_point(self):
        lidar_readings = self.Drone.lidar_and_wall_sim_with_gui.lidar_readings

        # we are actually all looking at relative readings
        # drone_location_meters = self.Drone.drone_location_meters

        min_distance = None
        closest_point = None

        for lidar_reading in lidar_readings:
            if min_distance is None or lidar_reading.total_relative_distance_m < min_distance:
                min_distance = lidar_reading.total_relative_distance_m
                closest_point = lidar_reading
        
        return (closest_point)

def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    drone_controller.Drone.update_lidar_readings()

    def on_key_press(event):
        K_YAW_INC = 30  # Define the increment value for the yaw change
        if event.keysym == "Right":  # Right arrow key
            drone_app.target_yaw = (drone_app.target_yaw + K_YAW_INC) % 360
            print("pressed")
        elif event.keysym == "Left":  # Left arrow key
            drone_app.target_yaw = (drone_app.target_yaw - K_YAW_INC + 360) % 360
            print("pressed")
        #elif event.keysym == "L" or event.keysym == "l":
        #    pass
        elif event.keysym == "Escape":
            pass
        else:
            if event.keysym == "Return":
                # Process the command
                command = drone_app.input_buffer.strip()
                drone_app.input_buffer = ""
                print(f"command received: {command}")

                if command.startswith("goto"):
                    print("command starts with goto")
                    try:
                        pass
                        # x, y = map(float, command[4:].split(","))
                        # drone_app.goto(x, y)
                    except ValueError:
                        print("Invalid goto command")
            elif event.keysym == "BackSpace":
                # Remove the last character from input_buffer unless it's empty
                if len(drone_app.input_buffer) > 0:
                    drone_app.input_buffer = drone_app.input_buffer[:-1]
            else:
                # Log the keys pressed
                drone_app.input_buffer += event.char

    # TODO GET THE KEY PRESSES AGAIN HERE
    # Bind the on_key_press function to the key press event
    #drone_app.lidar_and_wall_sim_with_gui.bind("<KeyPress>", on_key_press)
    # Set the focus to the canvas to receive keyboard events
    #drone_app.lidar_and_wall_sim_with_gui.focus_set()

    # Bind the on_command_entry_key_release method to the command_entry widget
    # drone_app.lidar_and_wall_sim_with_gui.command_entry.bind("<KeyRelease>", drone_app.lidar_and_wall_sim_with_gui.on_command_entry_key_release)


    
    while True:
        drone_controller.update_drone_velocity()

        # TODO: add drone_app.input_buffer
        # TODO: It is not getting updated on the key press, fix this
        # print(drone_app.input_buffer)
        
        print("A: {0:10.3f} D: {1:10.3f}, R: {2:10.3f}, P: {3:10.3f}, Y: {4:10.3f}".format(
            drone_controller.closest_point.lidar_angle_degrees,
            drone_controller.closest_point.total_relative_distance_m,
            drone_controller.velocity_x_setpoint,
            drone_controller.velocity_y_setpoint,
            drone_controller.error_yaw
        ))
        

        #drone_controller.Drone.set_attitude_setpoint(0, 0)
        #scaled_mouse_velocity = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        #drone_app.set_attitude_setpoint(scaled_mouse_velocity[0], scaled_mouse_velocity[1])
        
        time.sleep(timestep)

        drone_controller.Drone.update_location_meters(timestep)
        #print(f"iter {i}: Pre update_lidar_readings: ", type(drone_controller.Drone))
        drone_controller.Drone.update_lidar_readings()
        #lidar_readings = drone_controller.Drone.lidar_and_wall_sim_with_gui.lidar_readings
        # drone_controller.Drone.wipe_gui()
        # drone_controller.draw_perceived_wall()
        # drone_controller.Drone.lidar_and_wall_sim_with_gui.add_text(
        #     f'{drone_app.input_buffer}\n'
        #     f'Altitude: {drone_controller.Drone.drone_location_meters[2]:.1f}m\n'
        #     f'Mode: {"mode1"}')
        # drone_controller.Drone.update_gui()
        


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4)),
                Wall((0, 4), (0, 8)),
                Wall((0, 8), (6, 8)),
                Wall((6, 8), (6, -8)),
                Wall((6, -8), (0, -8)),
                Wall((0, -8), (0, -4)),
                Wall((0, -4), (-4, -4))   ]

    # Define the initial meters coordinates of the drone
    drone_location_meters = (0, 0, 0)

    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.05
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 90

    # Create a simulated drone object with simple physics
    # TODO note: simulated drone with the derivative is jumpy, this is OK, whatever
    #lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, lidar_noise_meters_standard_dev)
    # TODO pass in the lidar_and_wall_sim_with_gui to the drone object
    # TODO drone_location_meters is in the drone itself
    drone_app = Simulated_Drone_Simple_Physics(walls, drone_location_meters, drone_yaw_degrees, lidar_noise_meters_standard_dev)
    #drone_app = Simulated_Drone_Realistic_Physics(walls, drone_location_meters, drone_yaw_degrees, lidar_noise_meters_standard_dev)
    #drone_app = Real_Drone_Realistic_Physics(walls, drone_location_meters, drone_yaw_degrees, lidar_noise_meters_standard_dev)

    drone_controller = Drone_Controller(drone_app)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_controller.Drone,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    run_simulation(drone_app)
    #drone_controller.Drone.lidar_and_wall_sim_with_gui.mainloop()