# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Goals
The goal of this project is to design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.

The solution consists of the following files:

1. main.cpp: interfaces with the simulator and invokes the path planner.
2. planner.hpp: implements the path planner. Given the telemetry data provided from the simulator generates a new path for the car in world coordinates.
3. spline.h: cubic spline interpolation library by Tino Kluge used by the trajectory generator.
4. json.hpp: JSON library for C++ used to interface the simulator.

All components of the path planner are included in the main.cpp.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

The planner itself proceeds in five steps:
1. Trajectory generation: This basically is responsible to generate a path which the vehicle needs to follow.
2. Sensor fusion: Here, we use the provided sensor fusion to gather information about the other vehicles such as their speed and the lane they are driving on.
3. Speed control and collision avoidance: Acceleration and deceleration are defined to avoid violations and jerks to enable speed adjustments.
4. Lane switching and cruising: Conditions defined under which to switch lanes or keep driving at the current lane.
5. Behavior planning (especially lane choosing): strategies for setting a future lane to drive in. Cost functions are the key factor here. Low Cost, low traffic and highest average speed are the options available.

## Trajectory generation
Trajectory generation is a low level action which gets the car to move according to the lane lines. Conditions are defined to avoid driving outside of lanes or directly on the lane lines, bearing in mind the smooth steering. The code for generating trajectories is based on the spline library and is mostly adopted from the Project Q&A video with some minor changes. The code corresponding to trajectory generation can be found from approx. line 323 - 427 in main.cpp.

## Sensor fusion
We use sensor fusion to obtain information about the other vehicles on the road, such as their speed, their lanes and their distance to our vehicle. These type of information will be used in all the other components, such as speed control and lane selection.

## Speed control and collision avoidance
The speed of the leading vehicle is used over the sensor fusion vector in Frenet coordinates. On noticing the lead car position (s) within a certain safety distance, a decision check is made on whether a lane change is permissible or not. When its not possible, the vehicle is made to decelerate to match the velocity of the leading vehicle through a pre-defined deceleration rate. When the lane is clear again (upon say, lane changing) the vehicle is accelerated to the target speed.
A collision warning is established within 15m distance. A so called "EMERGENCY BRAKING" is enabled with a higher deceleration avoiding the violation limit.

(include gif here)

## Lane change maneuver
Lane change involves using sensor fusion information, primarily to determine the distance of other vehicles. The major distingishing factors are whether a lead vehicle is being followed, or driving on a lane with no traffic ahead.
If a vehicle is on the same lane as us and the distance falls below a certain safety distance (this could be done as function of speed dependancy), we check if a lane change is possible by measuring the distance of the vehicles on the adjacent lane(s). If, on the adjacent lane(s), no vehicle is within 30m in front or behind us, we perform the lane change. If a lane change is not possible due to other vehicles being present within the safety distance, we tell the vehicle to keep the current lane and to eventually slow down if needed.

## Behaviour Planning
Two main tactical planning strategies are implemented in this project, where the strategies are switched approximately halfway across the track.

"Keeping to the right as much as possible" is a highway traffic regulation, implemented in many european countries. Hence, to fulfill this legal requirement, it is always attempted to keep the vehicle to the right-most lane.

![](data/strategy1_short.gif)
This animation illustrates this strategy.

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

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

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

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

