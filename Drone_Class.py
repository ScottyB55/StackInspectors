from Drone_Realistic_Physics_Class import Drone_Realistic_Physics_Class
import math
import json
from enum import Enum

# Good idea: from collections import namedtuple


# Read config.json


def read_config(file_path):
    with open(file_path, "r", encoding="utf-8") as file:
        config = json.load(file)
    return config


config = read_config("config.json")
use_set_attitude = config["use_set_attitude"]


def rotate_point(x, y, angle_degrees):
    angle_radians = math.radians(angle_degrees)
    new_x = x * math.cos(angle_radians) - y * math.sin(angle_radians)
    new_y = x * math.sin(angle_radians) + y * math.cos(angle_radians)
    return new_x, new_y


# The drone has 2 key modes while in the air: Keyboard input only and wall follow
class DroneMode(Enum):
    KEYBOARD = 1
    WALL_FOLLOW = 2
    GROUND = 3
    TAKING_OFF = 4
    LANDING = 5


# We can switch the drone yaw control function in the air
# 1 is for yaw position control
# 2 is for yaw velocity control
class YawControlMode(Enum):
    POSITION = 1
    VELOCITY = 2


class Drone:
    """
    Base class for representing a drone with common methods and properties for all drones.
    """

    def __init__(self):
        """
        Initialize a Drone object.
        """
        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0
        self.target_hover_thrust = 0.5

        self.drone_location_meters = (0, 0, 0)

        self.drone_mode = DroneMode.GROUND
        self.yaw_control_mode = YawControlMode.VELOCITY

    def set_mode(self, mode):
        self.drone_mode = mode

    def get_mode(self):
        return self.drone_mode

    def get_lidar_readings_meters(self):
        pass

    def set_attitude_setpoint(
        self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5
    ):
        pass

    def takeoff(self):
        pass

    def land(self):
        pass

    def get_location_meters(self):
        return self.drone_location_meters


class Sam4_Drone(Drone):
    """
    Represents a real-life drone inheriting from the base Drone class.
    """

    def __init__(self):  # , drone_yaw_degrees
        super().__init__()
        # Make sure to put the connection string as a command line argument or pass it into the function
        self.drone = Drone_Realistic_Physics_Class()

        self.takeoff(target_altitude=1.5)

        # self.drone_location_meters = drone_location_meters

    def takeoff(self, target_altitude):
        self.drone_mode = DroneMode.TAKING_OFF
        self.drone.takeoff(target_altitude)
        self.drone_mode = DroneMode.KEYBOARD

    def get_current_yaw_angle(self):
        return self.drone.current_yaw_angle()

    def set_attitude_setpoint(
        self,
        target_roll,
        target_pitch,
        target_yaw=0,
        hover_thrust=0.5,
        yaw_control_mode=YawControlMode.VELOCITY,
    ):
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
        if use_set_attitude:
            gain = 2
            # print(f"set raw attitude yaw {target_yaw}")
            self.drone.set_attitude(
                target_roll * gain, -target_pitch * gain, target_yaw, hover_thrust
            )
            self.drone.ensure_transmitted()
        else:
            gain = 1

            if yaw_control_mode == YawControlMode.POSITION:
                # This works to set the absolute yaw
                # print(f"set yaw position {target_yaw}")
                self.drone.set_yaw(target_yaw)
                self.drone.ensure_transmitted()
                self.drone.set_velocity_body(
                    target_pitch * gain, target_roll * gain, 0.5 - hover_thrust
                )
            else:
                # print(f"set yaw velocity {target_yaw}")
                # Doesn't work if we set yaw=None, regardless of yaw_relative!
                if target_yaw > 180:
                    target_yaw = target_yaw - 360

                # target_yaw = -target_yaw #if it goes in the opposite direction as we'd expect

                # This works, but there is only 1 speed: all on or 0
                # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw, yaw_rate=target_yaw/30, yaw_relative=False)

                # This works, but there is only 1 speed: all on or 0
                # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw/30, yaw_rate=target_yaw/30, yaw_relative=False)

                # This works and there are multiple speeds, and the yaw rotation is in the opposite direction as expected
                # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw/300, yaw_rate=target_yaw/300, yaw_relative=False)

                # This works and there are multiple speeds, and the yaw rotation is in the same direction as expected
                # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw/300, yaw_rate=target_yaw/300, yaw_relative=True)

                # This works and there are multiple speeds, and the yaw rotation is in the same direction as expected
                # Try to get it in degrees per second. By setting to 10deg/sec and seeing it does a circle in 36 seconds
                target_yaw = target_yaw / 222
                self.drone.set_velocity_body(
                    target_pitch * gain,
                    target_roll * gain,
                    0.5 - hover_thrust,
                    yaw=target_yaw,
                    yaw_rate=target_yaw,
                    yaw_relative=True,
                )

                # This doesn't work, stays put regardless of orientation
                # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=0, yaw_rate=target_yaw/30, yaw_relative=False)

            # This works to set the yaw velocity, but not the exact yaw!
            # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw, yaw_rate=15, yaw_relative=False)

            # Doesn't work for both simultaneous, but works for set attitude or velocity alone
            """self.drone.set_attitude(target_roll=None, target_pitch=None, target_yaw=target_yaw, hover_thrust=None)
            self.drone.ensure_transmitted()
            self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust)"""

            # This doesn't work at all
            # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw, yaw_rate=None, yaw_relative=True)

            # This doesn't work at all
            # self.drone.set_velocity_body(target_pitch*gain, target_roll*gain, 0.5 - hover_thrust, yaw=target_yaw, yaw_rate=None, yaw_relative=False)

            self.drone.ensure_transmitted()

    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        # This is already 3 dimensional
        self.drone_location_meters = self.drone.current_location_meters()
        return self.drone_location_meters

    def land(self):
        self.drone_mode = DroneMode.LANDING
        self.drone.land()
        self.drone_mode = DroneMode.GROUND


class Simulated_Drone_Simple_Physics(Drone):
    """
    Represents a simulated drone with simple physics.

    Args:
        wall_start_meters (tuple): The meters coordinates of the starting point of the wall.
        wall_end_meters (tuple): The meters coordinates of the ending point of the wall.
        drone_location_meters (tuple): The meters coordinates of the drone.
        lidar_noise_meters_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self):
        super().__init__()
        # self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, lidar_noise_meters_standard_dev)
        self.drone_location_meters = (0, 0, 0)
        self.drone_yaw_degrees = 0

    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """

        K_YAW_CTRL = 33

        error = self.target_yaw - self.drone_yaw_degrees
        if abs(error) > 180:
            if error > 0:
                error -= 360
            else:
                error += 360

        self.drone_yaw_degrees += error * K_YAW_CTRL / 100 + 360
        self.drone_yaw_degrees %= 360

        # Rotate the target_roll and target_pitch by drone_yaw_degrees
        rotated_target_roll, rotated_target_pitch = rotate_point(
            self.target_roll, self.target_pitch, -self.drone_yaw_degrees
        )

        hover_increment = self.target_hover_thrust - 0.5

        # Update drone_location_meters
        self.drone_location_meters = tuple(
            a + b * timestep
            for a, b in zip(
                self.drone_location_meters,
                (rotated_target_roll, rotated_target_pitch, hover_increment),
            )
        )

        # print(self.drone_velocity)
        # self.drone_location_meters = tuple(a + b * timestep for a, b in zip(self.drone_location_meters, self.drone_velocity))
        # self.lidar_and_wall_sim_with_gui.drone = self.drone_location_meters
        # return self.drone_location_meters

    def get_current_yaw_angle(self):
        return self.drone_yaw_degrees

    def set_attitude_setpoint(
        self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5, yaw_mode=1
    ):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        # TODO: add functionality here for yaw and thrust
        self.target_roll = target_roll
        self.target_pitch = target_pitch
        self.target_yaw = target_yaw
        self.target_hover_thrust = hover_thrust
