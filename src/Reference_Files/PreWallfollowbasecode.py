# implementing a pre-wall follow mode is to constantly monitor the distance from the wall and switch to wall follow mode only when necessary.
import time
import math
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Set up the connection to the drone using Mavproxy
connection_string = 'udp:127.0.0.1:14550'
vehicle = connect(connection_string, wait_ready=True)

# Set up the LIDAR sensor
lidar = vehicle.rangefinder

# Set the threshold distance for pre-wall follow mode
threshold_distance = 2.0  # meters

# Set up the PID controller for wall follow mode
Kp = 0.5
Ki = 0.0
Kd = 0.0
last_error = 0.0
integral = 0.0

# Define the PID controller function
def pid_controller(error):
    global last_error, integral

    integral += error
    derivative = error - last_error
    output = Kp * error + Ki * integral + Kd * derivative

    last_error = error

    return output

# Define the pre-wall follow mode function
def pre_wall_follow_mode():
    global vehicle

    # Fly in a straight line until the threshold distance is crossed
    while lidar.distance > threshold_distance:
        vehicle.simple_goto(vehicle.location.global_relative_frame + (10, 0, 0))
        time.sleep(0.1)

    # Switch to wall follow mode once the threshold distance is crossed
    while lidar.distance <= threshold_distance:
        # Calculate the error between the current distance and the threshold distance
        error = lidar.distance - threshold_distance

        # Calculate the output of the PID controller
        output = pid_controller(error)

        # Set the yaw and pitch angles based on the output of the PID controller
        yaw_angle = math.degrees(output)
        pitch_angle = 0

        # Send the yaw and pitch commands to the drone
        msg = vehicle.message_factory.set_attitude_target_encode(
            0,  # time_boot_ms
            1,  # target system
            1,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame
            0b0000011111000111,  # type_mask (yaw and pitch)
            [0, 0, 0, 0],  # q
            0,  # body roll rate
            0,  # body pitch rate
            math.radians(yaw_angle),  # body yaw rate
            math.radians(pitch_angle),  # thrust
        )
        vehicle.send_mavlink(msg)
        time.sleep(0.1)

    # Switch back to normal flight mode once the threshold distance is crossed again
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = True

# Call the pre-wall follow mode function
pre_wall_follow_mode()

# Close the connection to the drone
vehicle.close()

#adjusting P,D
#
#
#
from pymavlink import mavutil
import keyboard

# Initialize gains
Kp = 0.0
Kd = 0.0

# Connect to SITL using MAVProxy
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Define callback function for key presses
def key_pressed(e):
    global Kp, Kd
    if e.name == 'p':   # increase P gain by 0.1
        Kp += 0.1
    elif e.name == 'P': # decrease P gain by 0.1
        Kp -= 0.1
    elif e.name == 'd': # increase D gain by 0.01
        Kd += 0.01
    elif e.name == 'D': # decrease D gain by 0.01
        Kd -= 0.01
    print(f'P gain: {Kp:.1f}, D gain: {Kd:.2f}')
    
    # Send updated gains to drone
    msg = master.mav.param_set_encode(
        target_system=1, target_component=mavutil.mavlink.MAV_COMP_ID_CONTROLLER,
        param_id=b'PID_P', param_value=Kp)
    master.mav.send(msg)
    msg = master.mav.param_set_encode(
        target_system=1, target_component=mavutil.mavlink.MAV_COMP_ID_CONTROLLER,
        param_id=b'PID_D', param_value=Kd)
    master.mav.send(msg)

# Register key callback function
keyboard.on_press(key_pressed)

# Run loop
while True:
    # Handle incoming MAVLink messages
    msg = master.recv_match()
    if msg:
        # Handle message here
        pass
