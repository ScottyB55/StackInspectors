### Data

Firstrun and Secondrun used the set attitude function, and experienced a high offset.
The yaw did properly set itself to face the wall.

## drone log files (.bin)

Open this online in a MavData viewer

## terminal output files (.txt)

Show the closest lidar point and target roll, pitch, yaw, and throttle

## s2lidar.py

This is an experiment script that we used in order to generate some data from the lidar.
We also used this to confirm that the lidar is working the way we expect it to.

This script essentially takes scans from the lidar for a set period of time and then uses
matplotlib to visualize the 360 scans.
