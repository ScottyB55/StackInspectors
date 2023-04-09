import numpy as np
import matplotlib.pyplot as plt

# Define Lidar filter function
def lidar_filter(data, direction):
    filtered_data = []
    for point in data:
        if point[1] > direction[0]*point[0] + direction[1]:
            filtered_data.append(point)
    return filtered_data

# Define line of best fit function
def line_of_best_fit(data):
    x = [point[0] for point in data]
    y = [point[1] for point in data]
    coefficients = np.polyfit(x, y, 1)
    line_function = np.poly1d(coefficients)
    return line_function(x), coefficients

# Continuously receive and process Lidar data
while True:
    # Receive Lidar data
    data = receive_lidar_data()

    # Filter Lidar data
    filtered_data = lidar_filter(data, direction=(1, 0))

    # Calculate line of best fit
    line_data, line_coefficients = line_of_best_fit(filtered_data)

    # Plot Lidar data and line of best fit
    plt.scatter(data[:,0], data[:,1])
    plt.plot(filtered_data[:,0], line_data, 'r')
    plt.show()
    
    #plot the drone’s setpoint velocity in terms of parallel & perpendicular components to the lidar line of best fit.
#import numpy as np

# assume we have the lidar line of best fit as a numpy array 'line'
# assume we have the drone's velocity setpoint as a numpy array 'velocity'

# calculate the parallel component of velocity
parallel_velocity = np.dot(velocity, line) / np.linalg.norm(line)

# calculate the perpendicular component of velocity
perpendicular_velocity = np.cross(velocity, line) / np.linalg.norm(line)

# plot the parallel and perpendicular components
import matplotlib.pyplot as plt

plt.plot(parallel_velocity, label='Parallel Component')
plt.plot(perpendicular_velocity, label='Perpendicular Component')
plt.legend()
plt.show()

#make the mouse only control the drone’s parallel component

#import numpy as np
#import matplotlib.pyplot as plt
#from pymavlink import mavutil

# Connect to the drone
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Define the wall line of best fit
x = np.linspace(0, 10, 11)
y = 2*x + 1

# Define the mouse control function
def get_mouse_control():
    mouse_pos = plt.ginput(1)
    parallel_vel = mouse_pos[0][0] - drone_pos[0]
    return parallel_vel

# Set the initial drone position
drone_pos = (5, 5)

# Initialize the plot
fig, ax = plt.subplots()
ax.plot(x, y, '-')

# Start the loop
while True:
    # Get the drone position from the flight controller
    msg = master.recv_match(type='LOCAL_POSITION_NED')
    if not msg:
        continue
    drone_pos = (msg.x, msg.y)

    # Get the mouse control input
    parallel_vel = get_mouse_control()
    
#Create another mostly independent controller to make the drone face the wall by adjusting the drone’s yaw
    
import math
import time
from pymavlink import mavutil
from PID import PID  # Import the PID class

# Initialize the PID controller with the desired parameters
kp = 1
ki = 0
kd = 0.1
yaw_pid = PID(kp, ki, kd, output_limits=(-1, 1))

# Connect to the MAVLink system and start the autopilot
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()

# Main control loop
while True:
    # Get the lidar data and calculate the line of best fit
    lidar_data = get_lidar_data()
    line_of_best_fit = calculate_line_of_best_fit(lidar_data)

    # Get the drone's position and calculate the distance to the line of best fit
    drone_position = get_drone_position()
    distance_to_line = calculate_distance_to_line(drone_position, line_of_best_fit)

    # Calculate the desired yaw angle based on the orientation of the line of best fit
    desired_yaw = math.atan(line_of_best_fit[1])

    # Calculate the current yaw angle of the drone
    current_yaw = get_drone_yaw()

    # Calculate the error between the desired and current yaw angles
    yaw_error = desired_yaw - current_yaw

    # Calculate the yaw velocity required to correct the error using the PID controller
    yaw_velocity = yaw_pid(yaw_error)

    # Set the yaw velocity of the drone
    set_yaw_velocity(yaw_velocity)

    # Wait for a short period before the next iteration
    time.sleep(0.1)
