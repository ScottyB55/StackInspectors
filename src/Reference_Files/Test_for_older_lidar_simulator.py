import unittest
from Reference_Files.V2_Lidar_Sim_Class import DroneLidar, lidar_reading_to_deltaxy
import math

class TestDroneLidar(unittest.TestCase):
    def assertAlmostEqualWithMessage(self, angle, actual, expected, delta=1e-6):
        if actual is None:
            self.assertIsNone(expected, f"Angle: {angle}, Actual: {actual}, Expected: {expected}")
        else:
            try:
                self.assertAlmostEqual(actual, expected, delta=delta)
            except AssertionError:
                raise AssertionError(f"Angle: {angle}, Actual: {actual}, Expected: {expected}")

    def test_get_lidar_readings_gps(self):
        # Test case 1
        wall_start_gps = (1+0, 1+1)
        wall_end_gps = (1+1, 1+1)
        drone_gps = (1+0, 1+0)
        drone_yaw_degrees = 0

        drone_lidar = DroneLidar(wall_start_gps, wall_end_gps, drone_gps, drone_yaw_degrees)
        lidar_readings = drone_lidar.get_lidar_readings_gps()

        # Add assertions to check if the lidar readings are as expected
        self.assertAlmostEqualWithMessage(0, lidar_readings[0], 1)
        self.assertAlmostEqualWithMessage(45, lidar_readings[45], math.sqrt(2))
        # self.assertAlmostEqualWithMessage(315, lidar_readings[315], math.sqrt(2))

        for angle in range(46, 359):
            self.assertAlmostEqualWithMessage(angle, lidar_readings[angle], None)

        # You can add more test cases with different input parameters
        # and expected lidar readings as needed.

    def test_lidar_reading_to_deltaxy(self):
        test_cases = [
            (45, 1, (0.7071067811865475, 0.7071067811865475)),
            (90, 1, (1, 0)),
            (180, 2, (0, -2)),
            (270, 3, (-3, 0)),
            (315, 1, (-0.7071067811865475, 0.7071067811865475))
        ]

        for angle, distance, expected in test_cases:
            delta_x, delta_y = lidar_reading_to_deltaxy(angle, distance)
            self.assertAlmostEqual(delta_x, expected[0], delta=1e-6, msg=f"Angle: {angle}, Distance: {distance}, Delta x: {delta_x}, Expected Delta x: {expected[0]}")
            self.assertAlmostEqual(delta_y, expected[1], delta=1e-6, msg=f"Angle: {angle}, Distance: {distance}, Delta y: {delta_y}, Expected Delta y: {expected[1]}")

if __name__ == '__main__':
    unittest.main()