# Stack-Inspectors

## UML Diagram Link

[View Diagram as PDF here](https://drive.google.com/file/d/1ueujWdafHSC29JfIKv2W1HGgGTog5tAg/view?usp=sharing)

## main424.py

```bash
git clone https://github.com/ScottyB55/StackInspectors.git --recursive
```

This is the main function that we run. Configure with config.json explained below. Follow these instructions to run:

[Stack-Inspectors Detailed Instructions Google Docs](https://docs.google.com/document/d/1zWLYlcgYcKWaxyjsA4dXIAKQVPLutofw7zy7St32ojE/edit?usp=sharing)

## py_rplidar_sdk

Our code depends on this submodule which we created.

Currently, the code will work on a Raspberry Pi 4.

\*You will need to recompile the py_rplidar_sdk code in order to test on different architectures (linux/mac/windows)

Run the following commands to compile the code

```bash
cd py_rplidar_sdk/
cmake .
make
```

This will generate a new s2lidar.so shared object file. This is what the python code will use in order to interface with the s2lidar.

## config.json

\*You will need to create a config.json after you clone the repository in order for the code to run.

`use_real_lidar`: set false if we want to use the simulated lidar

`use_gui`: set true if we want the GUI. Typically use this on SITL or simulated flights, but not on real flights

`use_mavproxy`: set true we want to do SITL or the real life drone. Set false if we want the pure python simulation

`target_distance`: set this to something like 5.0, this is how far the drone will follow the wall

`use_set_attitude`: set this to false so we use the set velocity function instead of the set attitude function. Set attitude has issues.

See example-config.json as a guide.

## Drone_Controller.py

The PID controller that allows us to follow the wall.

## Drone_Class.py

Allows us to easily switch between a MAVLink drone and a pure python simulated drone.

## Drone_Realistic_Physics_Class.py

The wrapper class around the MAVLink drone commands.

## GUI.py

Creates a GUI to display the walls and lidar points relative to the drone.

## Lidar_and_Wall_Simulator.py

Has functions to read the real & fake lidar, as well as to simulate the walls for the fake lidar.
