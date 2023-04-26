from Drone_Class import Sam4_Drone, Simulated_Drone_Simple_Physics, DroneMode, YawControlMode
from Drone_Controller import Drone_Controller
import time
import threading
from Lidar_and_Wall_Simulator import Wall, Lidar_and_Wall_Simulator
from GUI import GUI
import json
from sshkeyboard import listen_keyboard

# A global variale to increment or decrement velocities depending on key presses
roll_ctrl = 0
pitch_ctrl = 0
throttle_ctrl = 0
yaw_ctrl = 0

# How much we increment the velocity in meters per second, if the user presses awsd
key_press_delta = 0.2
# How much we increment the yaw velocity in degrees per second, if the user presses q or e
key_press_yaw_delta = 10

# Read the contents of config.json
def read_config(file_path):
    with open(file_path, "r") as file:
        config = json.load(file)
    return config
config = read_config("config.json")
use_gui = config["use_gui"]
use_mavproxy = config["use_mavproxy"]
target_distance = config["target_distance"]

# Instantiate a global drone object depending on use_mavproxy in the config.json file
if use_mavproxy:
    drone_inst = Sam4_Drone()
else:
    drone_inst = Simulated_Drone_Simple_Physics()

run_program = True

# Handle all of the keypress events. This works on SSH as well as
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
        yaw_ctrl = 0
    elif event == "l":
        print("l pressed")
        drone_inst.land()
        run_program = False
    elif event == "t":
        print("t pressed")
        drone_inst.takeoff(1.5)
    elif event == "f":
        if drone_inst.drone_mode == DroneMode.KEYBOARD:
            drone_inst.drone_mode = DroneMode.WALL_FOLLOW
            yaw_ctrl = 0
        else:
            drone_inst.drone_mode = DroneMode.KEYBOARD
        print("f pressed")
    elif event == "e":
        yaw_ctrl += key_press_yaw_delta
        if yaw_ctrl > 30:   # clamp
            yaw_ctrl = 30
        print("q pressed")
    elif event == "q":
        yaw_ctrl -= key_press_yaw_delta
        if yaw_ctrl < -30:
            yaw_ctrl -= 30
        print("e pressed")
    elif event == "m":
        if drone_inst.yaw_control_mode == YawControlMode.POSITION:
            drone_inst.yaw_control_mode = YawControlMode.VELOCITY
        else:
            drone_inst.yaw_control_mode = YawControlMode.POSITION
        print("m pressed")

def key_press_thread():
    listen_keyboard(on_press=key_on_press)

def run_simulation(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst=None):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1

    while run_program:

        # Wait and get the new lidar readings
        time.sleep(timestep)
        drone_inst.update_location_meters(timestep)
        lidar_and_wall_sim_inst.read_new_lidar_readings_angle_deg_dist_m(drone_inst)

        # Get the closest point to the drone
        closest_point_relative = lidar_and_wall_sim_inst.get_closest_point()

        # By default, the roll, pitch, yaw, and throttle is hover in place
        rpyt = [0.0, 0.0, 0.0, 0.5]
        # If we are in wall follow mode, calculate the target roll, pitch, yaw, and throttle using the PID from the lidar input
        if drone_inst.drone_mode == DroneMode.WALL_FOLLOW:
            rpyt = drone_controller_inst.get_target_drone_roll_pitch_yaw_thrust_pid(closest_point_relative)
        
        global pitch_ctrl, roll_ctrl, yaw_ctrl, throttle_ctrl
        
        # Define the maximum and minimum values for each element in rpyt
        MAX_ROLL = 0.4
        MIN_ROLL = -0.4
        MAX_PITCH = 0.4
        MIN_PITCH = -0.4
        MAX_THROTTLE = 0.6
        MIN_THROTTLE = 0.4

        # Add the control values to roll, pitch, yaw, and throttle (rpyt)
        rpyt[0] += roll_ctrl
        rpyt[1] += pitch_ctrl
        rpyt[2] += yaw_ctrl
        rpyt[3] += throttle_ctrl

        # Clamp the values of rpyt
        rpyt[0] = max(min(rpyt[0], MAX_ROLL), MIN_ROLL)
        rpyt[1] = max(min(rpyt[1], MAX_PITCH), MIN_PITCH)
        rpyt[3] = max(min(rpyt[3], MAX_THROTTLE), MIN_THROTTLE)

        # Set the new velocity setpoint
        drone_inst.set_attitude_setpoint(rpyt[0], rpyt[1], rpyt[2], rpyt[3], drone_inst.yaw_control_mode)

        # Update the GUI
        if (use_gui):
            GUI_inst.create_figure()
            GUI_inst.draw_drone()
            GUI_inst.draw_walls(walls)
            GUI_inst.draw_lidar_points(lidar_and_wall_sim_inst.get_lidar_readings_angle_deg_dist_m())
            GUI_inst.update_canvas()

        # Display the mode as a string on the terminal
        mode_string = "Follow" if drone_inst.drone_mode == DroneMode.WALL_FOLLOW else "Keys Only"
        yaw_mode = "Yaw Pos" if drone_inst.yaw_control_mode == YawControlMode.POSITION else "Yaw Vel"

        # Print the closest lidar point as well as the target roll, pitch, yaw, throttle, & other info on the terminal
        print("A: {0:10.3f} D: {1:10.3f}, R: {2:10.3f}, P: {3:10.3f}, Y: {4:10.3f}, T: {5:10.3f}, Mode: {6}, {7}".format(
            closest_point_relative.lidar_angle_degrees,
            closest_point_relative.total_relative_distance_m,
            rpyt[0],
            rpyt[1],
            rpyt[2],
            rpyt[3],
            yaw_mode,
            mode_string
        ))

if __name__ == '__main__':
    # The following wall & noise code is isnored if in the config file use_real_lidar is set to true

    # Define the starting and ending points of the simulated wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4))]
    # Define the standard deviation of the LIDAR noise in units of meters
    lidar_noise_meters_standard_dev = 0.03

    # Create a lidar object (which also can create a simulated wall)
    lidar_and_wall_sim_inst = Lidar_and_Wall_Simulator(walls, lidar_noise_meters_standard_dev)

    # Create a drone controller object
    drone_controller_inst = Drone_Controller(float(target_distance))

    # Start a thread to handle keyboard presses
    key_press_t = threading.Thread(target=key_press_thread)
    key_press_t.start()

    # If use_gui is set to true in the config file, we create a GUI
    if use_gui:
        GUI_inst = GUI()
    else:
        GUI_inst = None

    # Create a thread that runs the main loop. I'm not sure why we don't just have a while true loop here. It works, whatever.
    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the threads differently depending on whether we are using the GUI or not
    if use_gui:
        # Run the main event loop of the drone application (Tkinter GUI)
        GUI_inst.mainloop()
    else:
        # Wait for the move_drone_thread to complete
        move_drone_thread.join()