from Drone_Class import Simulated_Drone_Realistic_Physics, Sam4_Drone, Simulated_Drone_Simple_Physics, DroneMode
from Drone_Controller import Drone_Controller
import time
import threading
from Lidar_and_Wall_Simulator import Wall, Lidar_and_Wall_Simulator
from GUI import GUI
import keyboard


hover_thrust_range_fraction = 0.5

roll_ctrl = 0
pitch_ctrl = 0
throttle_ctrl = 0

key_press_time = 0.5
key_press_delta = 1

drone_inst = Sam4_Drone()
#drone_inst = Simulated_Drone_Simple_Physics()

run_program = True

def key_press_thread():
    global pitch_ctrl, roll_ctrl, drone_inst
    while True:
        event = keyboard.read_event()
        if event.name == "w":
            print("w pressed")
            pitch_ctrl = key_press_delta
            time.sleep(key_press_time)
            pitch_ctrl = 0
        elif event.name == "a":
            print("a pressed")
            roll_ctrl = -key_press_delta
            time.sleep(key_press_time)
            roll_ctrl = 0
        elif event.name == "s":
            print("s pressed")
            pitch_ctrl = -key_press_delta
            time.sleep(key_press_time)
            pitch_ctrl = 0
        elif event.name == "d":
            print("d pressed")
            roll_ctrl = key_press_delta
            time.sleep(key_press_time)
            roll_ctrl = 0
        elif event.name == "l":
            print("l pressed")
            drone_inst.land()
            run_program = False
        elif event.name == "t":
            print("t pressed")
            drone_inst.takeoff(3)



def run_simulation(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst=None):
    """
    Run the simulation of the drone, updating its position and displaying LIDAR data.

    Args:
        drone_app (Simulated_Drone_Simple_Physics): The drone application instance.
    """
    timestep = 0.1
    mouse_position_normalized_to_meters_velocity = 1

    drone_inst.update_location_meters(timestep)

    while run_program:
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

        global pitch_ctrl, roll_ctrl
        
        # Define the maximum and minimum values for each element in rpyt
        MAX_ROLL = 1
        MIN_ROLL = -1
        MAX_PITCH = 1
        MIN_PITCH = -1
        MAX_THROTTLE = 0.7
        MIN_THROTTLE = 0.3

        # Add the control values to rpyt
        rpyt[0] += roll_ctrl
        rpyt[1] += pitch_ctrl
        rpyt[3] += throttle_ctrl

        # Clamp the values of rpyt
        rpyt[0] = max(min(rpyt[0], MAX_ROLL), MIN_ROLL)
        rpyt[1] = max(min(rpyt[1], MAX_PITCH), MIN_PITCH)
        rpyt[3] = max(min(rpyt[3], MAX_THROTTLE), MIN_THROTTLE)


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
            print("A: {0:10.3f} D: {1:10.3f}, R: {2:10.3f}, P: {3:10.3f}, Y: {4:10.3f}".format(
            closest_point_relative.lidar_angle_degrees,
            closest_point_relative.total_relative_distance_m,
            rpyt[0],
            rpyt[1],
            rpyt[2]
            ))


if __name__ == '__main__':
    use_real_lidar = False
    use_gui = True  # Set this to False if you don't want to use the GUI

    # Define the starting and ending meters coordinates of the wall
    walls = [   Wall((-4, -4), (-4, 4)),
                Wall((-4, 4), (0, 4))]
    # Define the standard deviation of the LIDAR noise in meters units
    lidar_noise_meters_standard_dev = 0.1

    lidar_and_wall_sim_inst = Lidar_and_Wall_Simulator(use_real_lidar, walls, lidar_noise_meters_standard_dev)

    target_distance = input("Enter Target Distance: ")
    drone_controller_inst = Drone_Controller(float(target_distance))

    if use_gui:
        GUI_inst = GUI()
    else:
        GUI_inst = None

    # Start the keyboard listener thread
    key_press_t = threading.Thread(target=key_press_thread)
    key_press_t.start()

    # Start a new thread to run the simulation, updating the drone's position and LIDAR data
    move_drone_thread = threading.Thread(target=run_simulation, args=(use_gui, drone_inst, drone_controller_inst, lidar_and_wall_sim_inst, walls, GUI_inst,))
    # Set the thread as a daemon thread so it will automatically exit when the main program exits
    move_drone_thread.daemon = True
    # Start the simulation thread
    move_drone_thread.start()

    if use_gui:
        # Run the main event loop of the drone application (Tkinter GUI)
        GUI_inst.mainloop()
    else:
        # Wait for the move_drone_thread to complete
        move_drone_thread.join()
