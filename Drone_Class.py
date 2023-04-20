#from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized
import time
import threading
#from Lidar_and_Wall_Simulator import Lidar_and_Wall_Simulator
from Drone_Realistic_Physics_Class import Drone_Realistic_Physics_Class
import math

# Good idea: from collections import namedtuple

def rotate_point(x, y, angle_degrees):
    angle_radians = math.radians(angle_degrees)
    new_x = x * math.cos(angle_radians) - y * math.sin(angle_radians)
    new_y = x * math.sin(angle_radians) + y * math.cos(angle_radians)
    return new_x, new_y


class Drone:
    """
    Base class for representing a drone with common methods and properties for all drones.
    """
    def __init__(self):
        """
        Initialize a Drone object.
        """
        self.input_buffer = ""

        self.target_roll = 0
        self.target_pitch = 0
        self.target_yaw = 0
        self.target_hover_thrust = 0.5

    def get_lidar_readings_meters(self):
        """
        Get LIDAR readings for the drone in meters coordinates.
        """
        pass

    def set_attitude_setpoint(self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        pass

    def take_off(self):
        """
        Implement the take_off logic for the drone.
        """
        pass

    def land(self):
        """
        Implement the land logic for the drone.
        """
        pass

    # Add other common methods and properties for all drones

class Sam4_Drone(Drone):
    """
    Represents a real-life drone inheriting from the base Drone class.
    """
    def __init__(self): #, drone_yaw_degrees
        super().__init__()
        # Make sure to put the connection string as a command line argument or pass it into the function
        self.drone = Drone_Realistic_Physics_Class()

        self.takeoff(target_altitude=3)

        #self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, self, lidar_noise_meters_standard_dev)
        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(0,0)
        # self.drone_location_meters = drone_location_meters

    def takeoff(self, target_altitude):
        self.drone.takeoff(target_altitude)
    
    def get_current_yaw_angle(self):
        return self.drone.current_yaw_angle()

    def set_attitude_setpoint(self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5):
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
        gain = 2
        self.drone.set_attitude(target_roll*gain, -target_pitch*gain, target_yaw, hover_thrust)
    
    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        # This is already 3 dimensional
        self.drone_location_meters = self.drone.current_location_meters()
        self.lidar_and_wall_sim_with_gui.drone = self.drone_location_meters
        return self.drone_location_meters

class Real_Drone_Realistic_Physics(Drone):
    """
    Represents a real-life drone inheriting from the base Drone class.
    """
    def __init__(self, walls, drone_location_meters, drone_yaw_degrees, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        super().__init__()
        # Make sure to put the connection string as a command line argument or pass it into the function
        self.drone = Drone_Realistic_Physics_Class()
        self.drone_yaw_degrees = drone_yaw_degrees

        self.takeoff(target_altitude=3)

        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, self, lidar_noise_meters_standard_dev)
        self.drone_location_meters = drone_location_meters

    def takeoff(self, target_altitude):
        self.drone.takeoff(target_altitude)
    
    def get_current_yaw_angle(self):
        return self.drone.current_yaw_angle()

    def set_attitude_setpoint(self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5):
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
        gain = 2
        self.drone.set_attitude(target_roll*gain, -target_pitch*gain, target_yaw, hover_thrust)
    
    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        # This is already 3 dimensional
        self.drone_location_meters = self.drone.current_location_meters()
        self.lidar_and_wall_sim_with_gui.drone = self.drone_location_meters
        return self.drone_location_meters

class Simulated_Drone_Realistic_Physics(Drone):
    """
    Represents a simulated drone with realistic physics using SITL QGroundControl.
    """

    def __init__(self, walls, drone_location_meters, drone_yaw_degrees, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        super().__init__()
        # Make sure to put the connection string as a command line argument or pass it into the function
        self.drone = Drone_Realistic_Physics_Class()
        self.drone_yaw_degrees = drone_yaw_degrees

        self.takeoff(target_altitude=3)

        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, lidar_noise_meters_standard_dev)
        self.drone_location_meters = drone_location_meters

    def takeoff(self, target_altitude):
        self.drone.takeoff(target_altitude)
    
    def get_current_yaw_angle(self):
        return self.drone.current_yaw_angle()

    def set_attitude_setpoint(self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5):
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
        gain = 2
        self.drone.set_attitude(target_roll*gain, -target_pitch*gain, target_yaw, hover_thrust)
    
    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        # This is already 3 dimensional
        self.drone_location_meters = self.drone.current_location_meters()
        self.lidar_and_wall_sim_with_gui.drone = self.drone_location_meters
        return self.drone_location_meters


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
        #self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, lidar_noise_meters_standard_dev)
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
        rotated_target_roll, rotated_target_pitch = rotate_point(self.target_roll, self.target_pitch, -self.drone_yaw_degrees)

        hover_increment = self.target_hover_thrust - 0.5

        # Update drone_location_meters
        self.drone_location_meters = tuple(a + b * timestep for a, b in zip(self.drone_location_meters, (rotated_target_roll, rotated_target_pitch, hover_increment)))
        
        # print(self.drone_velocity)
        # self.drone_location_meters = tuple(a + b * timestep for a, b in zip(self.drone_location_meters, self.drone_velocity))
        # self.lidar_and_wall_sim_with_gui.drone = self.drone_location_meters
        # return self.drone_location_meters

    def get_current_yaw_angle(self):
        return self.drone_yaw_degrees

    def set_attitude_setpoint(self, target_roll, target_pitch, target_yaw=0, hover_thrust=0.5):
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


def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1

    while True:
        roll_pitch_setpoint_tuple = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        # using the defaults for yaw and throttle
        drone_app.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])
        time.sleep(timestep)

        drone_app.update_location_meters(timestep)
        drone_app.update_lidar_readings()
        drone_app.wipe_gui()
        drone_app.update_gui()


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    walls = [((1, -2), (1, 2)), ((1, -2), (3, -2))]
    # Define the initial meters coordinates of the drone
    drone_location_meters = (0.5, 0.5)
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Simulated_Drone_Simple_Physics(walls, drone_location_meters, lidar_noise_meters_standard_dev)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_app.lidar_and_wall_sim_with_gui.mainloop()
