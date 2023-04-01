from mouse_and_keyboard import mouse_relative_position_from_center_normalized, on_press
import sys
from Guided_Drone_Class import Guided_Drone, get_location_metres
import math
import time

# The code to get the drone connected
TARGET_ALTITUDE = 3
WAYPOINT_LIMIT = 1 #Make this at least 1, otherwise there can be issues!

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

drone.takeoff(target_altitude=TARGET_ALTITUDE, altitude_reach_threshold=0.95)

# save initial yaw to use as target
#target_yaw = math.degrees(vehicle.attitude.yaw)
run = 1

try:
    # Collect events until stopped
    from pynput import keyboard
    with keyboard.Listener(on_press=on_press) as listener:
        # Go 20 meters in the direction of the student ID angle
        Student_ID_Last_2_Digits = 43
        Student_ID_Angle = Student_ID_Last_2_Digits * 3
        Student_ID_DISTANCE = 20
        Student_ID_Dist_N_S = Student_ID_DISTANCE * math.cos(Student_ID_Angle*math.pi/180)
        Student_ID_Dist_E_W = Student_ID_DISTANCE * math.sin(Student_ID_Angle*math.pi/180)
        print("Going 20 meters with a heading of " + str(Student_ID_Angle) + " degrees from north clockwise:")
        print("Dist_N_S: " + str(Student_ID_Dist_N_S) + "Dist_E_W: " + str(Student_ID_Dist_E_W))

        currentLocation=drone.currentLocation()
        #						+North/-South	+East/-West
        targetLocation=get_location_metres(currentLocation, Student_ID_Dist_N_S, Student_ID_Dist_E_W, TARGET_ALTITUDE)
        drone.simple_goto(targetLocation)
        while (drone.distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
            time.sleep(0.1)
        time.sleep(0.5)

        # Now get our current location after traveling the student ID distance
        currentLocation=drone.currentLocation()

        # Now drive in a circle
        num_waypoints = 10
        radius = 10
        print(drone.vehicle.location.global_relative_frame)
        for i in range(1, num_waypoints + 1):
            angle_degrees = 90 + ((360 / num_waypoints) * i)  # Calculate the angle in degrees
            angle_radians = math.radians(angle_degrees)  # Convert the angle to radians

            x = radius * math.cos(angle_radians)
            y = radius * (-1 + math.sin(angle_radians))

            #						+North/-South	+East/-West
            targetLocation=get_location_metres(currentLocation, y, x, TARGET_ALTITUDE)
            drone.simple_goto(targetLocation)
            while (drone.distanceToWaypoint(targetLocation) > WAYPOINT_LIMIT):
                time.sleep(0.1)
            time.sleep(0.5)
        
            print(drone.vehicle.location.global_relative_frame)
            
except KeyboardInterrupt:
    print('exiting')

drone.rtl()