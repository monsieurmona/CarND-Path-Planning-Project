# CarND-Path-Planning-Project
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive) Implemented and Written by Mario Lüder

### Goals
The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit.

The path planner is provided with the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

### Implementation

#### Lane Selection
The speed of all lanes v<sub>next_car_in_lane</sub> in driving direction is derived by storing the speed of the next car for each lane in front of the ego vehicle. 

Additionally the distance d<sub>next_car</sub> between those cars and the ego vehicle are stored too. 

A safety distance is calculated based on the speed. This distance helps to avoid collision if something unpredictable happends. 1.5 seconds are a little more than the prediction interval, used in to steer the car. 

d<sub>safe</sub> = v<sub>car</sub> * 1.5s 

As we have the distance to the car in front we may calculate the maximum speed of this lane. 

v<sub>safe</sub> = d<sub>next_car</sub> / 1.5s

The minimum of v<sub>max</sub>, v<sub>safe</sub> and v<sub>next_car_in_lane</sub> defines the the lane speed v<sub>lane</sub>

The speeds of the lanes are sorted in descendant order.

The algorithm loops through this list and checks if it is possible to change lane till the current lane is in the list. 

Possible means that the distance to the next car (in the desired lane and all in between) is equal or larger than the safety distance d<sub>safe</sub> for the lane speed v<sub>lane</sub> and the distance to the car in back (in the desired lane and all in between) is equal or larger than the safety distance that this car would need.

#### Trajectory

The target speed v<sub>ego_predicted</sub> is calculated by (simple linear prediction - which is good enough for the simulator):

If a car is in front of the desired lane:

t<sub>prediction</sub> = 1s

d<sub>pred_next_car</sub> = v<sub>next_car_in_lane</sub> * t<sub>prediction</sub> + d<sub>next_car</sub><br>
d<sub>pred_ego_car</sub> = v<sub>ego</sub> * t<sub>prediction</sub><br>
d<sub>pred_diff</sub> = d<sub>pred_next_car</sub> - d<sub>pred_ego_car</sub><br>
v<sub>pred_safe</sub> = d<sub>pred_diff</sub> / 1.5s<br>
v<sub>ego_predicted</sub> = min(v<sub>pred_safe</sub>, v<sub>max</sub>, 0)<br>

If no car is there v<sub>ego_predicted</sub> is simply  v<sub>max</sub>.


The trajectory is calculated with the help of a spline. Two points on the the current lane, taken from the end of last prediction and three points on the desired lane are used to draw the curve. The distance to target points is calculated by multiplying the current speed with a factor. The factor 4.8 is selected to avoid too much of lateral acceleration.

 Based on this curve and speed v<sub>ego_predicted</sub> the points for the trajectory are computed, while speed is linearly interpolated between the last prediction and  v<sub>ego_predicted</sub>.
  
### Simulator
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Resource

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.

---

## Dependencies

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

