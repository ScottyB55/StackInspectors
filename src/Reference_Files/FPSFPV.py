from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import tkinter as tk
import math
import time
import sys
#import keyboard
import argparse

#Desired Takeoff Altitude
TARGET_ALTITUDE = 5
#Desired Takeoff Altitude Threshold
ALTITUDE_REACH_THRESHOLD = 0.95

#Sets yaw to specific heading, dir specifies the direction of yaw
def condition_yaw(heading, dir, relative=True):
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #Mavlink command being constructed
        0, #confirmation
        heading,    #yaw in degrees
        0,          #yaw speed
        dir,          #direction, 1 for yaw right, -1 for yaw left
        is_relative, #Determines relative or absolute angle
        0, 0, 0)    
    vehicle.send_mavlink(msg)

 #Sets the drone body velocity depending on vx, vy, vz
def set_velocity_body(vehicle, vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED, #Mavlink command being constructed
            0b0000111111000111, #Bitmask from sample code
            0, 0, 0,        #Positional arguements, setting to 0 maintains current position when not commanded otherwise
            vx, vy, vz,     #velocity parameters
            0, 0, 0,        
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
#Drone movement speed in m/s
drone_speed = 5    

#When space is pressed, increase altitude
def Alt_up(event):
    altitude = int(vehicle.location.global_relative_frame.alt)
    if (altitude + 5 < 50): #Ensures that even with some velocity from upwards movement the 50m theshold isnt passed
      set_velocity_body(vehicle, 0, 0, -1)
#When L_Ctrl is pressed, decrease altitude
def Alt_down(event):
    altitude = int(vehicle.location.global_relative_frame.alt)
    if (altitude > 5): #Ensures lower limit of 5m
      set_velocity_body(vehicle, 0, 0, 1) 
#Landing function, ended up not using it but would RTL when commanded
def Init_land():
    altitude = int(vehicle.location.global_relative_frame.alt)
    while (altitude >= 1): #Checks that altitude is acceptable to turn off vehicle
        altitude = int(vehicle.location.global_relative_frame.alt)
        time.sleep(1)
    print("Flight Complete!")
    vehicle.close()
#Detects key releases to zero out all velocities
def key_release(event):
    set_velocity_body(vehicle, 0, 0, 0)
    condition_yaw(0, 1) 

#Detects key presses and call relevant functions
def key(event):
    if event.char == event.keysym: #standard keys
        print(event.keysym)
        #if event.keysym == 'r':
        #    print("r pressed >> Set the vehicle to RTL")
        #    vehicle.mode = VehicleMode("RTL")
        #    Init_land()
        # Ended up not using this due to safety concerns, but would have handled RTL if enabled
        if event.keysym == 'w': #Forward movement
            set_velocity_body(vehicle, drone_speed, 0, 0) 
        elif event.keysym == 's': #Backwards movement
            set_velocity_body(vehicle,-drone_speed, 0, 0)
        elif event.keysym == 'a': #Leftwards movement
            set_velocity_body(vehicle, 0, -drone_speed, 0)
        elif event.keysym == 'd': #Rightwards movement
            set_velocity_body(vehicle, 0, drone_speed, 0)
        elif event.keysym == 'q': #Set yaw to left 
            condition_yaw(90, -1)
        elif event.keysym == 'e': #Set yaw to right
            condition_yaw(90, 1)
        #For some arbitrary reason Tkinter doesnt include these keys in the standard key set, thus the alternative implementation
        #elif event.keysm == 'space':
        #    set_velocity_body(vehicle, 0, 0, 1)
        #elif event.keysm == 'Control_L':
        #    set_velocity_body(vehicle, 0, 0 -1)    

#Sequence of commands to set up parser for target connection
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="vehicle connection target string")
args = parser.parse_args()

#acquire connection string
connection_string = args.connect

#Exit if not specified
if not connection_string:
    sys.exit('Specify Connection String')

#Connect to vehicle
print('connecting to vehicle on: %s' %connection_string)
vehicle = connect(connection_string, wait_ready=True)
print('connected to vehicle succesfully')

#Wait for safety pilot to arm drone before takeoff
print('Waiting for safety pilot to arm drone')
while not vehicle.armed:
    time.sleep(1)

#Once vehicle is armed
print('Vehicle is armed...')
vehicle.mode = VehicleMode("GUIDED")

#Takeoff to TARGET_ALTITUDE
print("Taking off!")
vehicle.simple_takeoff(TARGET_ALTITUDE)
while True:
    if vehicle.location.global_relative_frame.alt >= TARGET_ALTITUDE * ALTITUDE_REACH_THRESHOLD:
         break
    time.sleep(0.5)

#At this point vehicle is 5m in altitude and ready to start key intercepts
print("Taking Key Inputs!")

#Creates new tkinter object and binds keys to the new window
root = tk.Tk()
root.bind_all('<Key>', key)
root.bind('<space>', Alt_up)
root.bind('<Control_L>', Alt_down)
root.bind_all('<KeyRelease>', key_release)
root.mainloop() #Keeps the loop running until Tkinter is closed
while vehicle.armed:
    time.sleep(1) #Once landed and disarmed, closes out the vehicle
print("Flight Complete!")
vehicle.close()