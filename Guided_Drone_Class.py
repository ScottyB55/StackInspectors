from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import sys
import argparse
import math

# convert euler angles to quaternion to send over mavlink
def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    """
    Convert Euler angles to quaternion.

    Parameters:
        roll (float): Roll angle in degrees. Default is 0.0.
        pitch (float): Pitch angle in degrees. Default is 0.0.
        yaw (float): Yaw angle in degrees. Default is 0.0.

    Returns:
        A list containing four elements representing the quaternion [w, x, y, z].
    """
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

def rc_listener(self, name, message):
    """
    The purpose is to make sure that the drone is good before launching.
    Listens for RC_CHANNELS mavlink messages with the goal of determining when the RCIN_4 joystick
    has returned to center for two consecutive seconds.

    Parameters:
        name (str): The name of the message.
        message (pymavlink.dialects.v20.common.RC_CHANNELS): The RC_CHANNELS mavlink message.
    """
    global rcin_4_center
    rcin_4_center = (message.chan4_raw < 1550 and message.chan4_raw > 1450)
    # print(f"Message Chan4_Raw: {message.chan4_raw}")

class Guided_Drone:
    """
    A class for controlling a drone in guided mode using DroneKit.

    Attributes:
        vehicle (dronekit.Vehicle): The DroneKit vehicle object.
        rcin_4_center_once (bool): Indicates whether the RCIN_4 joystick has been centered once.
        rcin_4_center_twice (bool): Indicates whether the RCIN_4 joystick has been centered twice.
    """

    def __init__(self, connection_string):
        """
        Initializes the Guided_Drone object.

        Parameters:
            connection_string (str): The connection string for the drone.
        """
        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % connection_string)
        self.vehicle = connect(connection_string, wait_ready=False)
        print('Succesfully connected to vehicle')
        global rcin_4_center
        rcin_4_center = 0
        self.rcin_4_center_once = False
        self.rcin_4_center_twice = False

        self.vehicle.on_message('RC_CHANNELS')(rc_listener)

    def takeoff(self, target_altitude=3, altitude_reach_threshold=0.95):
        """
        Arms the drone and performs a takeoff to the specified altitude.

        Parameters:
            target_altitude (float): The desired altitude in meters to takeoff to. Default is 3 meters.
            altitude_reach_threshold (float): The fraction of the target altitude at which to break from the takeoff loop.
                Default is 0.95.

        Returns:
            None
        """
        
        if self.vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
            self.vehicle.mode = VehicleMode("ALT_HOLD")
            
        # Wait for pilot before proceeding
        print('Waiting for safety pilot to arm...')

        # Wait until safety pilot arms drone
        while not self.vehicle.armed:
            time.sleep(1)

        print('Armed...')
        self.vehicle.mode = VehicleMode("GUIDED")

        # Uses the rc_listener function to
        # Make sure that the drone is good before launching
        if self.vehicle.version.vehicle_type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            while not self.rcin_4_center_twice:
                if rcin_4_center:
                    if self.rcin_4_center_once:
                        self.rcin_4_center_twice = True
                    else:
                        self.rcin_4_center_once = True
                else:
                    self.rcin_4_center_once = False
                time.sleep(1)
                
            # Takeoff to short altitude
            print("Taking off!")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            while True:
                # Break just below target altitude.
                if self.vehicle.location.global_relative_frame.alt >= target_altitude * altitude_reach_threshold:
                    break
                time.sleep(0.5)

    def set_attitude(self, target_roll, target_pitch, target_yaw, hover_thrust):
        """
        Sets an attitude setpoint to the drone.

        Parameters:
            target_roll (float): Desired roll angle in degrees.
            target_pitch (float): Desired pitch angle in degrees.
            target_yaw (float): Desired yaw angle in degrees.
            hover_thrust (float): Desired thrust value for hovering. Should be between 0 and 1, 0.5 is no vertical velocity.

        Returns:
            None
        """
        # generate mavlink message to send attitude setpoint
        new_quat = to_quaternion(target_roll, target_pitch, target_yaw)
        # http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html#copter-commands-in-guided-mode-set-attitude-target
        msg = self.vehicle.message_factory.set_attitude_target_encode(
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
        self.vehicle.send_mavlink(msg)

    def rtl(self):
        """
        Lands the drone by switching the vehicle mode to LAND and waiting until the drone has landed.

        Parameters:
            None

        Returns:
            None
        """
        # switch vehicle mode to RTL
        self.vehicle.mode = VehicleMode("RTL")

        # wait until the drone has landed
        while self.vehicle.armed:
            time.sleep(0.5)

        # close the connection to the drone
        self.vehicle.close()