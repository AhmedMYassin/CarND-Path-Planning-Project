# Path Planning Project
Self-Driving Car Engineer Nanodegree Program

## Overview
The project aims to develop an algorithm for a vehicle to be able to drive in a simulated track with maximum speed `50 mph`, maximum acceleration `10m/s^2` and maximum `jerk 10m/s^3`. The vehicle has to drive safely and avoid any kind of collision and going out of track.

## Project Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


## Project Steps

### Vehicle Path Prediction
The waypoints in the vehicle nearby area are extracted from `highway_map.csv` and interpolated then using `Spline` library `50` points are generated to define the vehicle position within the next 90 meters. Vehicle path is defined according to the current vehicle lane, current vehicle speed and the target lane.

### Sensor Fusion Prediction
The sensors data are used to create predictions for the nearby vehicles. Vehicle speed and position are used to define its position with respect to the predicted vehicle position.

### Lane Selection
The main target is to keep the vehilce safe and make it drive at maximum available speed. Changing lanes requires making sure that the vehicle will be safe during this shift and no collision will happen. To decide to change the lane or not, we use the sensor fusion prediction output for vehicles in the target lane to make sure that there will be safe distance between our vehicle and other vehicles during and after the vehicle lane shift.

Total Run Time: `29 minutes`
Total Run Distance: `21 miles`

<img src="https://github.com/AhmedMYassin/Path-Planning-Project/blob/master/data/21_miles.png"> 
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project the goal is to safely navigate around a virtual highway with other traffic that is driving `+-10 MPH` of the `50 MPH` speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the `50 MPH` speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the `6946m` highway. Since the car is trying to go `50 MPH`, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than `10 m/s^3`.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

