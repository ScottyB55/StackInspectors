from dronekit import connect
from mouse_and_keyboard import mouse_relative_position_from_center_normalized, on_press

import sys
sys.path.append('/home/srburnett/Stack-Inspectors')
from Guided_Drone_Class import Guided_Drone
import math
import time

# The code to get the drone connected. Now testing to see if the github works

# Set up option parsing to get connection string and mission plan file
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string.")
args = parser.parse_args()

# aquire connection_string
connection_string = args.connect

# Exit if no connection string specified
if not connection_string:
    sys.exit('Please specify connection string')

# Instantiate Guided_Drone object
drone = Guided_Drone(connection_string)

drone.takeoff(target_altitude=3, altitude_reach_threshold=0.95)


target_roll = 0
target_pitch = 0

# save initial yaw to use as target
#target_yaw = math.degrees(vehicle.attitude.yaw)
# Set the period of the loop in seconds
iteration_time = .1
run = 1

try:
    # Collect events until stopped
    target_roll = 1
    target_pitch = 1
    from pynput import keyboard
    with keyboard.Listener(on_press=on_press) as listener:
        while run:
            mouse_x, mouse_y = mouse_relative_position_from_center_normalized()
            print(f"Mouse relative position from the center of the screen: x={mouse_x:.2f}, y={mouse_y:.2f} pixels")

            # hover_thrust (0-1 where 0.5 is no vertical velocity)
            drone.set_attitude(target_roll=mouse_x*10, target_pitch=mouse_y*10, target_yaw=0, hover_thrust=0.5)

            print(drone.vehicle.location.global_relative_frame)

            time.sleep(iteration_time)
            
except KeyboardInterrupt:
    print('exiting')

drone.rtl()