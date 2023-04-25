from Drone_Class import Simulated_Drone_Realistic_Physics, Sam4_Drone, Simulated_Drone_Simple_Physics, DroneMode
from Drone_Controller import Drone_Controller
import time
import threading
from Lidar_and_Wall_Simulator import Wall, Lidar_and_Wall_Simulator
from GUI import GUI
import keyboard
import json
from sshkeyboard import listen_keyboard

from enum import Enum

class DroneMode(Enum):
    KEYBOARD = 1
    WALL_FOLLOW = 2

current_mode = DroneMode.KEYBOARD

hover_thrust_range_fraction = 0.5

roll_ctrl = 0
pitch_ctrl = 0
throttle_ctrl = 0
yaw_ctrl = 0

key_press_time = 0.5
key_press_delta = 0.2
key_press_yaw_delta = 5

def read_config(file_path):
    with open(file_path, "r") as file:
        config = json.load(file)
    return config

config = read_config("config.json")
use_gui = config["use_gui"]
use_mavproxy = config["use_mavproxy"]
target_distance = config["target_distance"]

if use_mavproxy:
    drone_inst = Sam4_Drone()
else:
    drone_inst = Simulated_Drone_Simple_Physics()

run_program = True
def key_on_press(event):
    global pitch_ctrl, roll_ctrl, yaw_ctrl, throttle_ctrl, drone_inst
    if event == "w":
        print("w pressed")
        pitch_ctrl += key_press_delta
    elif event == "s":
        print("s pressed")
        pitch_ctrl += -key_press_delta
    elif event == "d":
        print("d pressed")
        roll_ctrl += key_press_delta
    elif event == "a":
        print("a pressed")
        roll_ctrl += -key_press_delta
    elif event == "[":
        print("[ pressed")
        throttle_ctrl -= key_press_delta
    elif event == "]":
        print("] pressed")
        throttle_ctrl += key_press_delta
    elif event == "z":
        print("z pressed")
        throttle_ctrl = 0
        roll_ctrl = 0
        pitch_ctrl = 0
    elif event == "l":
        print("l pressed")
        drone_inst.land()
        run_program = False
    elif event == "t":
        print("t pressed")
        drone_inst.takeoff(1.5)
    elif event == "f":
        global current_mode
        if current_mode == DroneMode.KEYBOARD:
            current_mode = DroneMode.WALL_FOLLOW
            yaw_ctrl = 0
        else:
            current_mode = DroneMode.KEYBOARD
        print("f pressed")
    elif event == "q":
        yaw_ctrl += key_press_yaw_delta
        if yaw_ctrl >= 360:
            yaw_ctrl -= 360
        print("q pressed")
    elif event == "e":
        yaw_ctrl -= key_press_yaw_delta
        if yaw_ctrl < 0:
            yaw_ctrl += 360
        print("e pressed")


def key_press_thread():
    listen_keyboard(on_press=key_on_press)

def run_simulation(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst=None):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1

    drone_inst.update_location_meters(timestep)

    while run_program:
        # Pure mouse input
        #roll_pitch_setpoint_tuple = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        #drone_inst.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])

        # Wait and get the new lidar readings
        time.sleep(timestep)
        drone_inst.update_location_meters(timestep)
        lidar_and_wall_sim_inst.read_new_lidar_readings_angle_deg_dist_m(drone_inst)


        # Calculate the new setpoint based on the lidar readings
        # Get the closest point to the drone
        closest_point_relative = lidar_and_wall_sim_inst.get_closest_point()

        rpyt = [0.0, 0.0, 0.0, 0.5]
        if current_mode == DroneMode.WALL_FOLLOW:
            # Calculate the target roll, pitch, yaw, and throttle from the PID only
            rpyt = drone_controller_inst.get_target_drone_roll_pitch_yaw_thrust_pid(drone_inst, closest_point_relative)

        global pitch_ctrl, roll_ctrl, yaw_ctrl, throttle_ctrl
        
        # Define the maximum and minimum values for each element in rpyt
        MAX_ROLL = 0.4
        MIN_ROLL = -0.4
        MAX_PITCH = 0.4
        MIN_PITCH = -0.4
        MAX_THROTTLE = 0.6
        MIN_THROTTLE = 0.4

        # Add the control values to rpyt
        rpyt[0] += roll_ctrl
        rpyt[1] += pitch_ctrl
        rpyt[2] += yaw_ctrl
        rpyt[3] += throttle_ctrl

        # Clamp the values of rpyt
        rpyt[0] = max(min(rpyt[0], MAX_ROLL), MIN_ROLL)
        rpyt[1] = max(min(rpyt[1], MAX_PITCH), MIN_PITCH)
        if (rpyt[2] >= 360):
            rpyt[2] -= 360
        rpyt[3] = max(min(rpyt[3], MAX_THROTTLE), MIN_THROTTLE)

        # Set the new velocity setpoint
        drone_inst.set_attitude_setpoint(rpyt[0], rpyt[1], rpyt[2], rpyt[3])

        if (use_gui):
            # Update the GUI
            GUI_inst.create_figure()
            GUI_inst.draw_drone()
            GUI_inst.draw_walls(walls)
            GUI_inst.draw_lidar_points(lidar_and_wall_sim_inst.get_lidar_readings_angle_deg_dist_m())
            GUI_inst.update_canvas()

        # Add the mode string based on the current_mode
        mode_string = "Follow" if current_mode == DroneMode.WALL_FOLLOW else "Keys Only"

        # Print the information to the console (or any other non-GUI logic)
        print("A: {0:10.3f} D: {1:10.3f}, R: {2:10.3f}, P: {3:10.3f}, Y: {4:10.3f}, T: {5:10.3f}, Mode: {6}".format(
            closest_point_relative.lidar_angle_degrees,
            closest_point_relative.total_relative_distance_m,
            rpyt[0],
            rpyt[1],
            rpyt[2],
            rpyt[3],
            mode_string
        ))
"""
def get_target_distance():
    global target_distance
    target_distance = float(input("Enter Target Distance: "))
"""
if __name__ == '__main__':
    #global use_gui # Set this to False if you don't want to use the GUI
    
    # Start the keyboard listener thread
    

    # Define the starting and ending meters coordinates of the wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4))]
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.03

    lidar_and_wall_sim_inst = Lidar_and_Wall_Simulator(walls, lidar_noise_meters_standard_dev)

    #global target_distance
    #target_distance = input("Enter Target Distance: ")
    drone_controller_inst = Drone_Controller(float(target_distance))
    #user_input_thread = threading.Thread(target=get_target_distance)
    #user_input_thread.start()
    key_press_t = threading.Thread(target=key_press_thread)
    key_press_t.start()
    #key_press_t.join()

    if use_gui:
        GUI_inst = GUI()
    else:
        GUI_inst = None

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    if use_gui:
        # Run the main event loop of the drone application (Tkinter GUI)
        GUI_inst.mainloop()
    else:
        # Wait for the move_drone_thread to complete
        move_drone_thread.join()
