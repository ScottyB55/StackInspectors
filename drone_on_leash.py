#!/usr/bin/env python3

from __future__ import print_function

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil




# The code to get the drone connected

# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 3
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95

# Launch
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

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=False)

print('Succesfully connected to vehicle')

"""
Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
has returned to center for two consecutive seconds.
"""
@vehicle.on_message('RC_CHANNELS')
def rc_listener(self, name, message):
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)


if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
    vehicle.mode = VehicleMode("ALT_HOLD")

# Wait for pilot before proceeding
print('Waiting for safety pilot to arm...')

# Wait until safety pilot arms drone
while not vehicle.armed:
    time.sleep(1)

print('Armed...')
vehicle.mode = VehicleMode("GUIDED")

if vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:

    rcin_4_center_once = False
    rcin_4_center_twice = False
    while not rcin_4_center_twice:
        if rcin_4_center:
            if rcin_4_center_once:
                rcin_4_center_twice = True
            else:
                rcin_4_center_once = True
        else:
            rcin_4_center_once = False
        time.sleep(1)
    
    # Takeoff to short altitude
    print("Taking off!")
    vehicle.simple_takeoff(TARGET_ALTITUDE)  # Take off to target altitude

    while True:
         # Break just below target altitude.
        if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
            break
        time.sleep(0.5)





# The Actual Code

run = 1
# thrust (0-1 where 0.5 is no vertical velocity)
hover_thrust = 0.6 
# save initial yaw to use as target
target_yaw = math.degrees(vehicle.attitude.yaw)
# Set the period of the loop in seconds
iteration_time = .1

# convert euler angles to quaternion to send over mavlink
# credit dronekit example: https://github.com/dronekit/dronekit-python/blob/master/examples/set_attitude_target/set_attitude_target.py
def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
	t0 = math.cos(math.radians(yaw * 0.5))
	t1 = math.sin(math.radians(yaw * 0.5))
	t2 = math.cos(math.radians(roll * 0.5))
	t3 = math.sin(math.radians(roll * 0.5))
	t4 = math.cos(math.radians(pitch * 0.5))
	t5 = math.sin(math.radians(pitch * 0.5))

	w = t0 * t2 * t4 + t1 * t3 * t5
	x = t0 * t3 * t4 - t1 * t2 * t5
	y = t0 * t2 * t5 + t1 * t3 * t4
	z = t1 * t2 * t4 - t0 * t3 * t5
	return [w, x, y, z]

try:
	while run:
		output = [1, 1]
		
		# generate mavlink message to send attitude setpoint
		new_quat = to_quaternion(output[0], output[1], target_yaw)
		# http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
		msg = vehicle.message_factory.set_attitude_target_encode(
			0, # time_boot_ms
			1, # target system
			1, # target component
			0b00000100,
			new_quat, # attitude (quaternion)
			0, # roll rate
			0, # pitch rate
			0, # yaw rate
			hover_thrust  # thrust (0-1 where 0.5 is no vertical velocity)
		)
		vehicle.send_mavlink(msg)

		time.sleep(iteration_time)
except KeyboardInterrupt:
	print('exiting')
	pass

vehicle.close()