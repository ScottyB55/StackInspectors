from Reference_Files.mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized, on_press

import sys
sys.path.append('/home/srburnett/Stack-Inspectors')
from Drone_Realistic_Physics_Class import Drone_Realistic_Physics_Class
import math
import time

# The code to get the drone connected. Now testing to see if the github works

# Make sure to put the connection string as a command line argument or pass it into the function
drone = Drone_Realistic_Physics_Class()

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
            # print(f"Mouse relative position from the center of the screen: x={mouse_x:.2f}, y={mouse_y:.2f} pixels")

            # hover_thrust (0-1 where 0.5 is no vertical velocity)
            drone.set_attitude(target_roll=mouse_x*10, target_pitch=-mouse_y*10, target_yaw=90, hover_thrust=0.5)

            print(drone.current_location_meters())
            print(drone.current_yaw_angle())

            time.sleep(iteration_time)
            
except KeyboardInterrupt:
    print('exiting')

drone.land()