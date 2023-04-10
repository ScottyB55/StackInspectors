from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized
import time
import threading
from Lidar_and_Wall_Simulator_With_GUI import Lidar_and_Wall_Simulator_With_GUI
from Drone_Realistic_Physics_Class import Drone_Realistic_Physics_Class

# Good idea: from collections import namedtuple


class Drone:
    """
    Base class for representing a drone with common methods and properties for all drones.
    """
    def __init__(self):
        """
        Initialize a Drone object.
        """
        pass

    def get_lidar_readings_meters(self):
        """
        Get LIDAR readings for the drone in meters coordinates.
        """
        pass

    def set_drone_velocity_setpoint(self, drone_velocity):
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


class Real_Drone_Realistic_Physics(Drone):
    """
    Represents a real-life drone inheriting from the base Drone class.
    """
    def __init__(self):
        """
        Initialize a Real_Drone_Realistic_Physics object.
        """
        super().__init__()


class Simulated_Drone_Simple_Physics(Drone):
    """
    Represents a simulated drone with simple physics.

    Args:
        wall_start_meters (tuple): The meters coordinates of the starting point of the wall.
        wall_end_meters (tuple): The meters coordinates of the ending point of the wall.
        drone_location_meters (tuple): The meters coordinates of the drone.
        lidar_noise_meters_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self, walls, drone_location_meters, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        super().__init__()
        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, drone_location_meters, lidar_noise_meters_standard_dev)
        self.drone_location_meters = drone_location_meters
    
    def update_lidar_readings(self):
        return self.lidar_and_wall_sim_with_gui.get_lidar_readings_angle_deg_dist_m()
    
    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        # print(self.drone_velocity)
        self.drone_location_meters = tuple(a + b * timestep for a, b in zip(self.drone_location_meters, self.drone_velocity))
        self.lidar_and_wall_sim_with_gui.drone_location_meters = self.drone_location_meters
        return self.drone_location_meters

    def set_drone_velocity_setpoint(self, drone_velocity):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        self.drone_velocity = drone_velocity
    
    def wipe_gui(self):
        self.lidar_and_wall_sim_with_gui.create_figure()

    def update_gui(self):
        self.lidar_and_wall_sim_with_gui.draw_drone()
        self.lidar_and_wall_sim_with_gui.draw_walls()
        self.lidar_and_wall_sim_with_gui.draw_lidar_points()
        # self.lidar_and_wall_sim_with_gui.draw_lidar_points_cluster()
        self.lidar_and_wall_sim_with_gui.update_canvas()


class Simulated_Drone_Realistic_Physics(Drone):
    """
    Represents a simulated drone with realistic physics using SITL QGroundControl.
    """

    def __init__(self, walls, drone_location_meters, lidar_noise_meters_standard_dev): #, drone_yaw_degrees
        super().__init__()
        # Make sure to put the connection string as a command line argument or pass it into the function
        self.drone = Drone_Realistic_Physics_Class()

        self.takeoff(target_altitude=3)

        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(walls, drone_location_meters, lidar_noise_meters_standard_dev)
        self.drone_location_meters = drone_location_meters

    def takeoff(self, target_altitude):
        self.drone.takeoff(target_altitude)
    
    def set_drone_velocity_setpoint(self, drone_velocity):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        gain = 2
        self.drone.set_attitude(target_roll=drone_velocity[0]*gain, target_pitch=-drone_velocity[1]*gain, target_yaw=0, hover_thrust=0.5)

    def update_lidar_readings(self):
        return self.lidar_and_wall_sim_with_gui.get_lidar_readings_angle_deg_dist_m()
    
    def update_location_meters(self, timestep):
        """
        Update the meters location of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's meters.
        """
        self.drone_location_meters = self.drone.current_location_meters()
        self.lidar_and_wall_sim_with_gui.drone_location_meters = self.drone_location_meters
        return self.drone_location_meters

    def wipe_gui(self):
        self.lidar_and_wall_sim_with_gui.create_figure()

    def update_gui(self):
        self.lidar_and_wall_sim_with_gui.draw_drone()
        self.lidar_and_wall_sim_with_gui.draw_walls()
        self.lidar_and_wall_sim_with_gui.draw_lidar_points()
        self.lidar_and_wall_sim_with_gui.update_canvas()


def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1
    while True:
        drone_app.set_drone_velocity_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))
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