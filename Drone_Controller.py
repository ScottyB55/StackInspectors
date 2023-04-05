"""
Chat GPT Ideas with some Editing

An effective approach for a drone to follow the closest wall for inspecting smokestacks would be to combine lidar data with 
a wall-following algorithm. Here's an outline of the steps involved:
    1. Preprocess lidar data: Filter and cluster the lidar data to identify the nearest wall points. 
    You can use the DBSCAN clustering method mentioned in the previous answer or any other suitable clustering technique.

    2. Estimate the wall's orientation: Perform linear regression on the nearest wall points to estimate the wall's orientation. 
    This will give you a line of best fit representing the wall's orientation. 
    Alternatively, you can also use techniques like RANSAC for a more robust estimation in the presence of noise.

    3. Calculate the drone's distance from the wall's line of best fit. Take this distance, and feed it back into the drone's
    orientation rotation (attitude) parallel to the wall's line of best fit. The user's mouse input should control the drone's
    orientation rotation (attitude) perpendicular to the wall line of best fit to allow the drone to move up and down the wall.

    4. Control the drone's position and orientation: Implement a PID controller or another suitable control algorithm 
    to steer the drone towards the target position while maintaining the desired distance from the wall. 
    The controller should handle both the drone's position and orientation to ensure it remains parallel to the wall.

    5. Monitor and update: Continuously update the drone's position, orientation, and lidar data. 
    If the drone approaches a corner or an obstacle, adjust its trajectory accordingly. 
    You can use additional sensors, such as cameras or ultrasonic sensors, to enhance the drone's perception of its surroundings.

    6. Safety measures: Implement safety measures like collision avoidance and failsafe mechanisms to ensure the drone's 
    safe operation in the smokestack environment.

This approach should allow the drone to follow the closest wall while inspecting smokestacks. 
You can further improve the performance by fusing data from multiple sensors and implementing advanced control algorithms.

Another thing to add: we want two controllers
1. One that makes the drone face the wall
2. One that makes 
"""

from Drone import Drone

class Drone:
    def __init__(self, Drone):
        self.Drone = Drone
        self.target_distance = 10   # the target distance between the drone and the wall
        self.target_angle = 90      # the target angle between the drone
    
    def run(self):
        # Get the lidar data
        self.Drone.get_lidar_readings_gps()

        # Optional: factor the drone's roll & pitch into the lidar data

        # Optional: cluster on the lidar data

        # TODO: get the line of best fit for the lidar data

        # Control Loop 1: adjust the drone's target_yaw to make the drone face towards the wall
        #   1. Calculate the angle between the line where the drone is facing & the lidar line of best fit
        #   2. This angle should be 90 degrees. Pass the error difference through a control function & feed back to the drone's target_yaw

        # Control Loop 2: adjust the drone's roll & pitch to make the drone maintain a constant distance from the wall
        #   1. Calculate the closest distance between the drone and the lidar line of best fit
        #   2. Establish some sort of matrix relationship as follows:
        #       Input: target angles parallel & perpendicular to the lidar line of best fit
        #       Output: target_roll and target_pitch for the drone
        #   3. Set the parallel input component to the difference between the desired distance & the actual distance
        #   4. Set the perpendicular input component to the mouse x coordinates

        # Additional Control: remember that there is an additional hover_thrust input that controls how much the drone moves up or down
        #   Set this proportional to the mouse y coordinates

        # Use something like the set_attitude() function for self.Drone.set_drone_velocity()
