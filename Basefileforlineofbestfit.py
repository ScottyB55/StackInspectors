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
