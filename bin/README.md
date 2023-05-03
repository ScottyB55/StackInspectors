# Executables

## Our main function that runs on the drone or simulated drone

main424.py

Our project uses the main424.py as an executable.
This means that the entry point of our program is in this file.

## C++ File to communicate with the Lidar

This is the shared object file that we generated with our C++ file
We referenced this shared object file from our python code to run the lidar program
rplidar.so

The shared object file is compiled using Cmake and is called s2lidar.so
