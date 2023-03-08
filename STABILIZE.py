#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
usage: python square_off.py --connect <*connection_string>
This script connects to the drone and waits until armed. When armed it will takeoff
to 3m altitude, then navigate a 10x10 meter square. At each corner of the square the drone
will wait for 5 seconds.
"""

from __future__ import print_function

import math
import time
import sys
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil

import getch

# Size of square in meters
SQUARE_SIZE = 10
# Desired altitude (in meters) to takeoff to
TARGET_ALTITUDE = 3
# Portion of TARGET_ALTITUDE at which we will break from takeoff loop
ALTITUDE_REACH_THRESHOLD = 0.95
# Maximum distance (in meters) from waypoint at which drone has "reached" waypoint
# This is used instead of 0 since distanceToWaypoint funciton is not 100% accurate
WAYPOINT_LIMIT = 1
# Variable to keep track of if joystick to arm has returned to center
rcin_4_center = False





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
vehicle.mode = VehicleMode("STABILIZE")#"GUIDED")

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




# # Set target altitude to 5 meters
# target_altitude = 5

# # Take off to target altitude
# vehicle.simple_takeoff(target_altitude)

# # Wait until the vehicle has reached target altitude
# while True:
#     current_altitude = vehicle.location.global_relative_frame.alt
#     if current_altitude >= target_altitude * 0.95:
#         break
#     # Wait for 0.1 seconds
#     time.sleep(0.1)

# # Takeoff to short altitude
# print("Target Altitude Achieved!")



# Function to handle keyboard input
def get_key():
    # Wait for key press
    key = getch.getch()
    
    # If the key is an arrow key, return the direction
    if key == '\x1b':  # arrow keys are preceded by an escape character
        getch.getch()  # ignore the next character, which is '['
        arrow = getch.getch()
        if arrow == 'A':
            return 'up'
        elif arrow == 'B':
            return 'down'
        elif arrow == 'C':
            return 'right'
        elif arrow == 'D':
            return 'left'
    # Otherwise, return None
    else:
        return None

# Set initial roll and pitch values
roll = 0
pitch = 0

# Loop to read keyboard input and control the drone
while True:
    # Get keyboard input
    #key = get_key()
    
    # Update roll and pitch based on keyboard input
    # if key == 'up':
    #     pitch += 0.1
    # elif key == 'down':
    #     pitch -= 0.1
    # elif key == 'right':
    #     roll += 0.1
    # elif key == 'left':
    #     roll -= 0.1
    pitch = 1
    roll = 0
    
    # Limit roll and pitch to between -1 and 1
    roll = max(min(roll, 1), -1)
    pitch = max(min(pitch, 1), -1)
    
    # # Set the vehicle attitude
    # vehicle.attitude.roll = roll
    # vehicle.attitude.pitch = pitch
    # vehicle.attitude.yaw = 0  # fixed yaw

    # Set the vehicle attitude
    vehicle.attitude.roll = roll
    vehicle.attitude.pitch = pitch
    vehicle.attitude.yaw = 0  # fixed yaw

    # Calculate the thrust value based on the desired pitch
    thrust = math.cos(pitch)

    # Create a MAVLink message to set the attitude and thrust
    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        vehicle._master.mav.srcSystem,  # target_system
        vehicle._master.mav.srcComponent,  # target_component
        1,  # type_mask (only set pitch)
        [pitch, 0, 0, thrust],  # q (attitude quaternion), thrust
        0,  # body roll rate
        0,  # body pitch rate
        0,  # body yaw rate
        0   # target system
    )

    # Send the MAVLink message to the drone
    vehicle.send_mavlink(msg)

    # Send the updated attitude to the vehicle
    vehicle.flush()

    # Wait for 0.1 seconds
    time.sleep(0.1)