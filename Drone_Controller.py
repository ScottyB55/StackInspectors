class Drone_Controller:
    def __init__(self, target_distance):
        self.target_distance = target_distance

        # Meters per second per meter of error
        self.Kp_roll_pitch = 1
        self.Kd_roll_pitch = 0

        # Degrees per second per degree of error
        self.Kp_yaw = 1

        self.velocity_x_setpoint = 0
        self.velocity_y_setpoint = 0
        self.distance_error = None
    
    def get_target_drone_roll_pitch_yaw_thrust_pid(self, closest_point_relative):
        # Roll & Pitch PID

        # Find the displacement between the drone and the closest point
        delta_x = closest_point_relative.x_relative_distance_m
        delta_y = closest_point_relative.y_relative_distance_m
        distance = closest_point_relative.total_relative_distance_m

        # Scale the deltax and deltay to a unit vector
        delta_x_unit = delta_x / distance
        delta_y_unit = delta_y / distance

        # Calculate the proportional and derivative error
        derivative_error = 0
        distance_error_prev = self.distance_error
        self.distance_error = distance - self.target_distance
        if (distance_error_prev != None):
            derivative_error = self.distance_error - distance_error_prev

        # Calculate the PID setpoint in-line with the direction towards the closest point
        # This way the PID works even if the yaw orientation is off
        self.velocity_x_setpoint = delta_x_unit * (self.Kp_roll_pitch * self.distance_error + self.Kd_roll_pitch * derivative_error)
        self.velocity_y_setpoint = delta_y_unit * (self.Kp_roll_pitch * self.distance_error + self.Kd_roll_pitch * derivative_error)


        # Yaw PID

        # Get the angle of the closest lidar point in degrees
        setpoint_yaw = closest_point_relative.lidar_angle_degrees
        # Move from [0 to 360] to [-180 to 180] so it includes a diretion & proprer amplitude error
        if (setpoint_yaw > 180):
            setpoint_yaw -= 360
        
        # Clamp it to 30 degrees worth of error
        if setpoint_yaw > 30:
            setpoint_yaw = 30
        elif setpoint_yaw < -30:
            setpoint_yaw = -30
        
        # PID, feed back the error into the new setpoint
        setpoint_yaw = setpoint_yaw * self.Kp_yaw


        # Return the PID-calculated roll, pitch, yaw, and throttle velocity setpoint (0.5 throttle means hover)
        return [self.velocity_x_setpoint, self.velocity_y_setpoint, setpoint_yaw, 0.5]