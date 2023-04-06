from Drone_Class import Simulated_Drone_Realistic_Physics
import time
import threading
from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized

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

        drone_app.update_location_meters()
        drone_app.update_lidar_readings()
        drone_app.update_gui()


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    wall_start_meters = (0, -2)
    wall_end_meters = (0, 2)
    # Define the initial meters coordinates of the drone
    drone_location_meters = (0.5, 0.5)
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1
    # Define the initial yaw angle of the drone in degrees (not used in this example)
    drone_yaw_degrees = 0

    # Create a simulated drone object with simple physics
    drone_app = Simulated_Drone_Realistic_Physics(wall_start_meters, wall_end_meters, drone_location_meters, lidar_noise_meters_standard_dev)

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(drone_app,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    # Run the main event loop of the drone application (Tkinter GUI)
    drone_app.lidar_and_wall_sim_with_gui.mainloop()