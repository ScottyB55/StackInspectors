from dronekit import connect

import sys
sys.path.append('/home/srburnett/Stack-Inspectors')
from Guided_Drone_Class import Guided_Drone
import math
import time

# The code to get the drone connected

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



# save initial yaw to use as target
#target_yaw = math.degrees(vehicle.attitude.yaw)
# Set the period of the loop in seconds
iteration_time = .1
run = 1

try:
    while run:
        # hover_thrust (0-1 where 0.5 is no vertical velocity)
        drone.set_attitude(target_roll=1, target_pitch=0, target_yaw=0, hover_thrust=0.5)
        time.sleep(iteration_time)

except KeyboardInterrupt:
    print('exiting')

drone.rtl()