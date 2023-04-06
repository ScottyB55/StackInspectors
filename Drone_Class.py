from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized
import time
import threading
from Lidar_and_Wall_Simulator_With_GUI import Lidar_and_Wall_Simulator_With_GUI

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

    def get_lidar_readings_gps(self):
        """
        Get LIDAR readings for the drone in GPS coordinates.
        """
        pass

    def set_drone_velocity(self, drone_velocity):
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
        wall_start_gps (tuple): The GPS coordinates of the starting point of the wall.
        wall_end_gps (tuple): The GPS coordinates of the ending point of the wall.
        drone_gps (tuple): The GPS coordinates of the drone.
        lidar_noise_gps_standard_dev (float): The standard deviation of the LIDAR noise.
    """

    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev): #, drone_yaw_degrees
        super().__init__()
        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev)
        self.drone_gps = drone_gps
    
    def update_physics(self, timestep):
        """
        Update the physics of the drone based on the given timestep.

        Args:
            timestep (float): The duration of the timestep for updating the drone's physics.
        """
        self.drone_gps = tuple(a + b * timestep for a, b in zip(self.drone_gps, self.drone_velocity))
        self.lidar_and_wall_sim_with_gui.drone_gps = self.drone_gps

    def set_drone_velocity(self, drone_velocity):
        """
        Set the velocity of the drone.

        Args:
            drone_velocity (tuple): A tuple containing the x and y components of the drone's velocity.
        """
        self.drone_velocity = drone_velocity
    
    def update_gui(self):
        self.lidar_and_wall_sim_with_gui.create_figure()
        self.lidar_and_wall_sim_with_gui.draw_drone()
        self.lidar_and_wall_sim_with_gui.draw_wall()
        self.lidar_and_wall_sim_with_gui.draw_lidar_points()
        self.lidar_and_wall_sim_with_gui.update_canvas()


class Simulated_Drone_Realistic_Physics(Drone):
    """
    Represents a simulated drone with realistic physics using SITL QGroundControl.
    
    """

    def __init__(self, wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev): #, drone_yaw_degrees
        super().__init__()
        self.lidar_and_wall_sim_with_gui = Lidar_and_Wall_Simulator_With_GUI(wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev)
        self.drone_gps = drone_gps


def run_simulation(drone_app):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_gps_velocity = 0.4
    while True:
        drone_app.update_gui()
        
        drone_app.set_drone_velocity(tuple(x * mouse_position_normalized_to_gps_velocity for x in mouse_relative_position_from_center_normalized()))

        time.sleep(timestep)
        drone_app.update_physics(timestep)


if __name__ == '__main__':
    # Define the starting and ending GPS coordinates of the wall
    wall_start_gps = (0, -1)
    wall_end_gps = (0, 1)
    # Define the initial GPS coordinates of the drone
    drone_gps = (0.5, 0.5)
    # Define the standard deviation of the LIDAR noise in GPS units
    lidar_noise_gps_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Simulated_Drone_Simple_Physics(wall_start_gps, wall_end_gps, drone_gps, lidar_noise_gps_standard_dev)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_app.lidar_and_wall_sim_with_gui.mainloop()