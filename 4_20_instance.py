from Drone_Class import Simulated_Drone_Realistic_Physics, Sam4_Drone, Simulated_Drone_Simple_Physics
from Drone_Controller import Drone_Controller
import time
import threading
#from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized
from mouse_and_keyboard_helper_functions import on_press, on_release, start_listening
from pynput import keyboard
from Lidar_and_Wall_Simulator import Wall, LidarReading, Lidar_and_Wall_Simulator
from GUI import GUI
from mouse_and_keyboard_helper_functions import mouse_relative_position_from_center_normalized

def run_simulation():
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1
    while True:
        roll_pitch_setpoint_tuple = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        #roll_pitch_setpoint_tuple = tuple(0,0)
        # using the defaults for yaw and throttle
        #drone_app.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])
        drone_inst.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])
        # drone_app.set_attitude_setpoint(tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized()))
        time.sleep(timestep)

        drone_inst.update_location_meters(timestep)
        lidar_and_wall_sim_inst.read_new_lidar_readings_angle_deg_dist_m(drone_inst)
        
        # GUI Update
        GUI_inst.create_figure()
        GUI_inst.draw_drone()
        GUI_inst.draw_walls(walls)
        GUI_inst.draw_lidar_points(lidar_and_wall_sim_inst.get_lidar_readings_angle_deg_dist_m())
        GUI_inst.update_canvas()


if __name__ == '__main__':
    # Define the starting and ending meters coordinates of the wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4))]
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1

    lidar_and_wall_sim_inst = Lidar_and_Wall_Simulator(False, walls, lidar_noise_meters_standard_dev)

    # Create a simulated drone object with simple physics
    #drone_inst = Sam4_Drone()
    drone_inst = Simulated_Drone_Simple_Physics()

    drone_controller_inst = Drone_Controller()
    
    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=())
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    GUI_inst = GUI()
    # Run the main event loop of the drone application (Tkinter GUI)
    GUI_inst.mainloop()