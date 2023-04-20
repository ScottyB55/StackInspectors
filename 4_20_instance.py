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
import userdistancegui

hover_thrust_range_fraction = 0.5

def run_simulation(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst=None):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1

    drone_inst.update_location_meters(timestep)

    while True:
        # Pure mouse input
        #roll_pitch_setpoint_tuple = tuple(x * mouse_position_normalized_to_meters_velocity for x in mouse_relative_position_from_center_normalized())
        #drone_inst.set_attitude_setpoint(roll_pitch_setpoint_tuple[0], roll_pitch_setpoint_tuple[1])

        # Wait and get the new lidar readings
        time.sleep(timestep)
        drone_inst.update_location_meters(timestep)
        lidar_and_wall_sim_inst.read_new_lidar_readings_angle_deg_dist_m(drone_inst)


        # Calculate the new setpoint based on the lidar readings
        # Get the closest point to the drone
        closest_point_relative = lidar_and_wall_sim_inst.get_closest_point()
        # Calculate the target roll, pitch, yaw, and throttle from the PID only
        rpyt = drone_controller_inst.get_target_drone_roll_pitch_yaw_thrust_pid(drone_inst, closest_point_relative)

        roll_ctrl = 0
        pitch_ctrl = 0
        throttle_ctrl = 0 # hover_thrust_setpoint = 0.5 + mouse_y * hover_thrust_range_fraction / 2

        rpyt[0] += roll_ctrl
        rpyt[1] += pitch_ctrl
        rpyt[3] += throttle_ctrl

        # TODO Clamp the values
        
        
        # Set the new velocity setpoint
        drone_inst.set_attitude_setpoint(rpyt[0], rpyt[1], rpyt[2], rpyt[3])

        if (use_gui):
            # Update the GUI
            GUI_inst.create_figure()
            GUI_inst.draw_drone()
            GUI_inst.draw_walls(walls)
            GUI_inst.draw_lidar_points(lidar_and_wall_sim_inst.get_lidar_readings_angle_deg_dist_m())
            GUI_inst.update_canvas()
        else:
            # Print the information to the console (or any other non-GUI logic)
            print("Drone position:", drone_inst.get_location_meters())
            print("Closest LIDAR point:", closest_point_relative)


if __name__ == '__main__':
    use_gui = True  # Set this to False if you don't want to use the GUI

    # Define the starting and ending meters coordinates of the wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4))]
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1

    lidar_and_wall_sim_inst = Lidar_and_Wall_Simulator(False, walls, lidar_noise_meters_standard_dev)

    # Create a simulated drone object with simple physics
    #drone_inst = Sam4_Drone()
    drone_inst = Simulated_Drone_Simple_Physics()
    
    target_distance = userdistancegui.distance_gui()  # the target distance between the drone and the wall
    print(f"Target Distance: {target_distance}")
    drone_controller_inst = Drone_Controller(target_distance)

    if use_gui:
        GUI_inst = GUI()
    else:
        GUI_inst = None

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    if use_gui:
        # Run the main event loop of the drone application (Tkinter GUI)
        GUI_inst.mainloop()

# That worked, thank you. Print out the additional code that needs to be adjusted or added so that the 