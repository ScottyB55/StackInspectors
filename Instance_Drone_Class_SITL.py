from Drone_Class import Simulated_Drone_Realistic_Physics
from Drone_Class import Sam4_Drone
import time
import threading
#from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized
from mouse_and_keyboard_helper_functions import on_press, on_release, start_listening
from pynput import keyboard

global key_roll
global key_yaw
global key_pitch
def on_press(key):
    print("onpress started")
    try:
        if key.char == 'w':
            print('W pressed')
            key_pitch = 1
        elif key.char == 'a':
            print('A pressed')
            key_roll = -1
        elif key.char == 's':
            print('S pressed')
            key_pitch = -1
        elif key.char == 'd':
            print('D pressed')
            key_roll = 1
    except AttributeError:
        pass

def keyboard_listener():
    print("keyboard thread started")
    with keyboard.Listener(on_press=on_press) as listener:
        listener.join()

# create the keyboard listener thread
thread = threading.Thread(target=keyboard_listener)
thread.daemon = True # set the thread as a daemon so it will exit when the program exits
thread.start()


def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1
    while True:
        #roll_pitch_setpoint_tuple = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        roll_pitch_setpoint_tuple = tuple(0,0)
        # using the defaults for yaw and throttle
        #drone_app.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])
        drone_app.set_attitude_setpoint(key_roll, key_pitch)
        # drone_app.set_attitude_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))
        time.sleep(timestep)

        drone_app.update_location_meters()
        drone_app.update_lidar_readings()
        drone_app.wipe_gui()
        drone_app.update_gui()


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    walls = [((1, -2), (1, 2)), ((1, -2), (3, -2))]
    # Define the initial meters coordinates of the drone
    drone_location_meters = (0.5, 0.5)
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Sam4_Drone(0, drone_location_meters, 0)
    
    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_app.lidar_and_wall_sim_with_gui.mainloop()