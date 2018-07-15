# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
---

## Objective
In this project the goal was to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization and sensor fusion data is provided, there is also a sparse map list of waypoints around the highway. The car tries to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. It is considered that other cars will try to change lanes too. Hitting other cars is avoided at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car drives safely around the complete highway loop which is 6946m long. Since the car is trying to go 50 MPH, it takes a little over 5 minutes to complete 1 loop. Also the car does not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

---

## Simulator input
Here is the data provided from the Simulator to the C++ Program

### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop. The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates
["y"] The car's y position in map coordinates
["s"] The car's s position in frenet coordinates
["d"] The car's d position in frenet coordinates
["yaw"] The car's yaw angle in the map
["speed"] The car's speed in MPH

### Previous path data given to the Planner
 
["previous_path_x"] The previous list of x points previously given to the simulator
["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value
["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

### Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

---

## How the rubrics are met

The following chapter descripes how the rubrics are met. 

### Overview

The pipeline to navigate savely around the virtual highway is split into three parts.

* a potential trajectory for each lane will be generated
* each of the trajectories will be evaluated with different cost functions to determin the best trajectory
* based on the costs of the best trajectory the velocity will be adjusted


### Trajectory generation

The trajectories are generated in line 362 to 468 of the main.c file.

A really helpful resource for creating smooth trajectories was using [splines](http://kluge.in-chemnitz.de/opensource/spline/). This single header file is used to setup a spline over a long distance (150m). Five anchor points (x and y points) generate a trajectory. The first two anchor points are the last two points of the previous trajectory. This makes sure that the new trajectory fits in seamlessly with the previous trajectory. 
The further anchor points are set at a distance of 50m, 100m and 150m (Frenet distance S). These three anchor points describe the coordinates which the trajectory should pass.
To make sure that a lane change over two lanes does not generate to much jerk, it is important to use the full 150m range to do the lane change. is done in two anchor point steps to avoid to much jerk

When five anchor points are defined, the car reference angle will be shifted to 0 degrees. This avoids the spline to get vertical. Shifting the with the current global map angle it makes sure the spline goes in x direction ( main.c - lines 426 / 427).

Once a spline is defined, I generate 150 x and y points. Since the distance between these points are defining the resulting velocity, it has to be considered. By linearizing the trajectory, the distance (150m) is devided into N points

```C++
double N = (target_dist / (0.02 * v_dest / 2.24));
double x_point = x_add + (target_x)/N;
double y_point = s(x_point);
```

With this formular, a velocity can be applied. The default velocity is 49.5mph.

At the end of this pipeline, x and y values for the potential trajectories are prepared. Each trajectory represents one lane (0 = left lane, 1 = middle lane, 2 = right lane)



### Cost functions
The next step is to determine, which of the generated trajectories / lanes is the best. Therefore ollowing cost functions are defined:


#### Costs if the lane is not safe
This function is defined from line 168 - 182. It defines a hitbox around our car (wide and length of the car). The costs for this trajectory will be increased of there is any other car around us.

These costs are configured with a high value. This means, that our model will avoid lane changes, if there are other vehicles.

In this functions the current ego frenet position (S value) is used. For each other car (sensor fusion data) and each lane the function checks if it is in a critical distance (range of 20m) of our car.


#### Costs for distance to the next car
This cost function (line 184 - 199 / main.c) helps to determine for each lane, in which distance the next car would occur. A distance of more than 150m is not penaltied with costs. The cost value between 0m and 150m is linear.

This function helps to decide which is the safest lane in the future.

#### Costs for other cars speed
This function (line 201 - 217 / main.c) adds cost per each lane, depending on the speed difference between the ego car and other cars. There costs are applied for cars in middle range (75m).

Using the velocity data of other cars lead to a better lane choice. It helps to keep the own velocity higher. In case that a lane change is not possible, the own velocity will be decreased.


#### Costs for lane changing
To avoid high accelerations and jerk, unnecessary lane changes should be avoided. Therefore a cost function (line 252 - 264 / main.c) for lane changing is implemented. Input parameters are "current lane" and "lane to check".
This function will work as a threshold, since the current lane will return costs of 0 in this function, and other lanes will charged.

A double lane change in one calculated trajectory (e.g. changing from lane 0 to 2) is allowed. But this is a more dangerous maneuver and could result in a collision or in a too high acceleration. Therefore the costs for such maneuvers are higher (double of a single lane change). But since there are some situations, in which a double lane change is helpful, and passing the middle lane is safe, a double lane change is allowed.


#### Costs if trajectory hits other cars in future (75 to 150 meters)
It is planned to compare the ego trajectory with an other cars trajectory. It may occurs that there will happen a collision with an other car in the planed trajectory. This collision may not be recognized early enough by only checking the short range distances.

This function is not implemented yet, but since the path planning works quite well with the other functions, it does not matter.


### Velocity adjustment
After all cost functions are used for each potential lane trajectory, the best cost lane will be choosen. Depending on the costs of the best rated lane, an additional velocity adjustment will be used. If the best cost value is very high, the ego car will slow down. If the best cost value is slow, the ego car try to speed up to nearly 50mph.

### Further configurations
The current weights, costs and further configurations are the following:

```
const double WEIGHT_COSTS_NEAR_APROACH = 10.0;
const double WEIGHT_COSTS_FOR_TRAJECTORY_CHANGE = 2.0;
const double REDUCE_VELOCITY_COST_THRESHOLD = 1.0;
const double WEIGHT_COSTS_IS_LANE_CURRENTLY_SAFE = 10.0;
const double WEIGHT_COSTS_DISTANCE_TO_CARS = 1.0;
const double WEIGHT_COSTS_SLOWER_VELOCITY = 4.0;

const int NUM_LANES = 3;

const int FAR_RANGE_IN_METERS = 150;
const int SHORT_RANGE_IN_METERS = 15;
const int MID_RANGE_IN_METERS = 75;
const double MAX_SPEED_MPH = 49.5;
```

---

## Project setup

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).


### Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

### Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

### Code Style

Code mainly sticks to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

