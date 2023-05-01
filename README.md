# Stack-Inspectors

## UML Diagram Link
https://lucid.app/lucidchart/471de1d5-ed9d-45fb-8df6-cf9c8afeb3d9/edit?view_items=U_EpCjbWtDTw&invitationId=inv_0f3e19bf-d86f-44ed-846d-c3ccd14a5509

## main424.py
This is the main function that we run. Configure with config.json explained below. Follow these instructions to run:
https://docs.google.com/document/d/1zWLYlcgYcKWaxyjsA4dXIAKQVPLutofw7zy7St32ojE/edit?usp=sharing

## config.json
"use_real_lidar": set false if we want to use the simulated lidar
"use_gui": set true if we want the GUI. Typically use this on SITL or simulated flights, but not on real flights
"use_mavproxy": set true we want to do SITL or the real life drone. Set false if we want the pure python simulation
"target_distance": set this to something like 5.0, this is how far the drone will follow the wall
"use_set_attitude": set this to false so we use the set velocity function instead of the set attitude function. Set attitude has issues.

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