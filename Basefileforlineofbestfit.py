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
