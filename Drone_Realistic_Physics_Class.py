from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import sys
import argparse
import math
import threading


"""
Includes launching and landing!
"""

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

def get_location_metres(original_location, dNorth, dEast, altitude):
    """
    Returns a LocationGlobal object containing the longitude/latitude `dEast` and `dNorth` metres from the 
    specified `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobalRelative(newlon, newlat, altitude)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

class Drone_Realistic_Physics_Class:
    """
    A class for controlling a drone in guided mode using DroneKit.

    Attributes:
        vehicle (dronekit.Vehicle): The DroneKit vehicle object.
        rcin_4_center_once (bool): Indicates whether the RCIN_4 joystick has been centered once.
        rcin_4_center_twice (bool): Indicates whether the RCIN_4 joystick has been centered twice.
    """

    def __init__(self, connection_string=None):
        """
        Initializes the Drone_Realistic_Physics_Class object.

        When instantiating this object, instantiate like this and the connection string command-line arguments
        will be covered in this class (don't worry, just instantiate like this below)
        my_instance = MyClass()

        TODO: I don't think the functionality works if the connection_string is provided, but I'm not sure

        Parameters:
            connection_string (str): The connection string for the drone.
        """
        if connection_string is None:
            connection_string = self.parse_args()
        self.connection_string = connection_string

        # Connect to the Vehicle
        print('Connecting to vehicle on: %s' % self.connection_string)
        self.vehicle = connect(self.connection_string, wait_ready=False)
        print('Succesfully connected to vehicle')
        global rcin_4_center
        rcin_4_center = 0
        self.rcin_4_center_once = False
        self.rcin_4_center_twice = False

        self.vehicle.on_message('RC_CHANNELS')(rc_listener)
    
    @staticmethod
    def parse_args():
        parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
        parser.add_argument('--connect', help="Vehicle connection target string.")
        args = parser.parse_args()

        connection_string = args.connect

        if not connection_string:
            sys.exit('Please specify connection string')

        return connection_string
    
    def distanceToWaypoint(self, coordinates):
        """
        Returns distance between vehicle and specified coordinates
        """
        distance = get_distance_metres(self.vehicle.location.global_frame, coordinates)
        return distance

    def simple_goto(self, targetLocation):
        """
        A wrapper for the simple_goto function of the self.vehicle instance.

        Parameters:
            target_location (LocationGlobalRelative): The target location (latitude, longitude, altitude).
        """
        self.vehicle.simple_goto(targetLocation)

    def current_location_gps(self):
        """
        Retrieves the current location of the vehicle as a global-relative frame.

        The global-relative frame is a coordinate system with the origin at the home location, where
        the vehicle was armed. The coordinates are in latitude, longitude, and altitude relative
        to the home location.

        Returns:
            mavlink.LocationGlobalRelative: The current location of the vehicle in a global-relative frame.
        """
        return self.vehicle.location.global_relative_frame

    def current_location_meters(self):
        """
        Retrieves the current location of the vehicle in meters relative to the home location.

        The home location is where the vehicle was armed. The coordinates are in x, y, and z
        relative to the home location, where x corresponds to latitude, y corresponds to longitude,
        and z corresponds to altitude.

        Returns:
            tuple: A tuple containing the x (latitude), y (longitude), and z (altitude) coordinates
                   of the vehicle's location in meters relative to the home location.
        """
        # Get the current location in global-relative frame
        location_global_relative = self.vehicle.location.global_relative_frame

        # Convert latitude and longitude to meters relative to home location
        y = (location_global_relative.lat - self.vehicle.home_location.lat) * 111139
        x = (location_global_relative.lon - self.vehicle.home_location.lon) * 111139 * math.cos(math.radians(location_global_relative.lat))
        z = location_global_relative.alt

        return x, y, z
    
    def current_yaw_angle(self):
        """
        Retrieves the current yaw angle of the vehicle in degrees.

        Returns:
            float: The current yaw angle of the vehicle in degrees.
        """
        # Get the current yaw angle in radians
        yaw_radians = self.vehicle.attitude.yaw

        # Convert yaw angle to degrees
        yaw_degrees = math.degrees(yaw_radians)

        return yaw_degrees
    
    def takeoff(self, target_altitude=3, altitude_reach_threshold=0.5):
        """
        Arms the drone and performs a takeoff to the specified altitude.

        Parameters:
            target_altitude (float): The desired altitude in meters to takeoff to. Default is 3 meters.
            altitude_reach_threshold (float): The fraction of the target altitude at which to break from the takeoff loop.
                Default is 0.9.

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
            print(f"Taking off to {target_altitude} and threshold of {target_altitude - altitude_reach_threshold}")
            self.vehicle.simple_takeoff(target_altitude)  # Take off to target altitude

            count = 0
            while count < 30:
                # Break just below target altitude.
                
                if self.vehicle.location.global_relative_frame.alt >= target_altitude - altitude_reach_threshold:
                    break
                time.sleep(0.5)
                count = count + 1

            if (count == 30):
                print(f"Takeoff Failed. Altitude: {self.vehicle.location.global_relative_frame.alt}")


    #Sets yaw to specific heading, dir specifies the direction of yaw
    def set_yaw(self, heading, relative=False, max_yaw_speed=0, dir=1):
        if relative:
            is_relative = 1
        else:
            is_relative = 0
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #Mavlink command being constructed
            0, #confirmation
            heading,       #yaw in degrees
            max_yaw_speed, #yaw speed
            dir,           #direction, 1 for yaw right, -1 for yaw left
            is_relative,   #Determines relative or absolute angle
            0, 0, 0)    
        
        self.vehicle.send_mavlink(msg)

    def set_velocity_body(self, vx, vy, vz, yaw=None, yaw_rate=None, yaw_relative=False):
        """
        Sets the drone body velocity depending on vx, vy, vz, and optionally yaw and yaw_rate.
        
        :param vx: X-axis velocity (forward is positive)
        :param vy: Y-axis velocity (right is positive)
        :param vz: Z-axis velocity (down is positive)
        :param yaw: Yaw angle in degrees (optional)
        :param yaw_rate: Yaw rate in degrees per second (optional)
        :param relative: If True, the yaw angle is relative to the current heading; otherwise, it's an absolute value.
        """
        # Create a MAVLink message for setting position target in local NED frame
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
                0,
                0, 0,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                0b0000111111000111,
                0, 0, 0,
                vx, vy, vz,
                0, 0, 0,
                0, 0)

        # If yaw is provided, update the yaw value and type mask
        if yaw is not None:
            msg.yaw = yaw
            msg.type_mask |= (1 << 9)  # Set bit 9 to use yaw angle
        else:
            msg.type_mask &= ~(1 << 9)  # Clear bit 9 to not use yaw angle

        if yaw_relative:
            msg.type_mask |= (1 << 12)
        else:
            msg.type_mask &= ~(1 << 12)

        # If yaw_rate is provided, update the yaw_rate value and type mask
        if yaw_rate is not None:
            msg.yaw_rate = yaw_rate
            msg.type_mask &= ~(1 << 10)  # Clear bit 10 to not ignore the yaw rate
        else:
            msg.type_mask |= (1 << 10)  # Set bit 10 to ignore the yaw rate

        # Send the MAVLink message
        self.vehicle.send_mavlink(msg)
    
    def ensure_transmitted(self):
        """
        Ensures that all MAVLink messages sent to the vehicle are transmitted
        by calling the `flush()` method on the vehicle object.

        :param vehicle: The vehicle object to flush the messages for.
        """
        self.vehicle.flush()

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
        Returns the drone to the launch position by switching the vehicle mode to RTL and waiting until the drone has landed.

        Parameters:
            None

        Returns:
            None
        """
        # Switch vehicle mode to RTL (Return to Launch)
        self.vehicle.mode = VehicleMode("RTL")

        # Wait until the drone has landed
        while self.vehicle.armed:
            time.sleep(0.5)

        # Close the connection to the drone
        self.vehicle.close()

    def land(self):
        """
        Lands the drone by switching the vehicle mode to LAND and waiting until the drone has landed.

        Parameters:
            None

        Returns:
            None
        """
        # Switch vehicle mode to LAND
        self.vehicle.mode = VehicleMode("LAND")

        # Wait until the drone has landed
        while self.vehicle.armed:
            time.sleep(0.5)

        # Close the connection to the drone
        self.vehicle.close()