
# Self-Driving Car Engineer Nanodegree Program - Path Planning Project 

![Screenshot](https://github.com/cazacov/CarND-Path-Planning-Project/blob/master/_img/screenshot.jpg?raw=true)
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Code structure
* src/main.cpp - Program entry point
    - instantiates other classes of the program
    - gets data from simulator and forwards it to PathPlanner
    - gets the calculated path from the PathPlanner and returns it back to simulator
* src/pathplanner.cpp - This is the most interesting part of the program that makes decisions and chooses the best trajectory. Uses several helper classes.
* src/maptransformer.cpp - Helper class for:
    - Coordinate transformations XY <-> Frenet
    - Converting the d coordinate to lane number and back
    - Waypoint helper functions.   
* src/speed_helper.cpp - Helper class to calculate acceleration profiles
    - Finds best acceleration profile given initial and target speed and considering maximum allowed acceleration and jerk
    - Estimates last speed, acceleration, x, y and yaw given the previous trajectory points.    
* src/trajectory_helper.cpp - Helper class to calculate possible movement trajectories
    - Finds trajectory given acceleration profile and initial and target lanes
    - Convert trajectory to way points
* src/collision_helper.cpp - Helper class to check if given trajectory is valid
    - Checks for collisions
    - Checks for speed and acceleration limits
    - Makes adjustment to highway map looped after 6945.554 meters
* src/spline.cpp; src/math_helper.h - Third party libraries for spline and polynomial interpolation

## Bird's eye view of the path planning algorithm

1. Get data from the simulator.
2. Looking at previous way points estimate the velocity and position at the last point. These values are then used as initial data for further path planning.  
3. Calculate acceleration profile from initial speed to the target speed.
4. Prepare ordered list of target lane candidates. Staying in the current lane has higher priority (comfort for passengers) than lane changing.
5. For every target lane candidate generate a smooth trajectory continuing the previous path.
6. Validate trajectories checking for collisions and speed/acceleration limits.
7. If there is a valid trajectory, then we are done! Generate path points and pass them to the simulator.
8. If none of trajectory candidates is valid, reduce target speed a little and go to step 3.
9. If no valid trajectory can be found and target speed in step 8 is decreased below the limit (10 m/s), then keep the current lane. That can happen if another car violates the traffic rules and changes the lane just before our car. In that situation keeping same lane and constant low speed is considered better than chaotic reaction of algorithm put in "impossible" initial conditions.    

## Planning horizon

The algorithm always plans for 3 seconds into the future trying to find the best trajectory that smoothly continues the previous one. To make reaction faster it preserves only first 12 points of the old trajectory and starts generating new waypoints from position 13. 12 points are enough for smooth transition from the old into new trajectory.

### Acceleration profile

Let's say we want to accelerate from the velocity __v0__ to __v1__ in __t__ seconds and want acceleration be 0 at the end (car reaches constant speed). Given initial acceleration __a0__ and maximal allowed acceleration and jerk, the graph of acceleration as a function of time should look like the following:

![Acceleration profile](https://github.com/cazacov/CarND-Path-Planning-Project/blob/master/_img/acceleration-profile.png?raw=true)

The graph starts with acceleration = a0 at time = 0 and ends with a = 0 at time = t. The yellow area between the graph and time axis is equal to the total velocity change __dv = v1 - v0__. To be comfortable for passengers the acceleration graph should also be inside the box limited by max allowed acceleration and have slope less than the maximal allowed jerk (green area).

Given initial values it's possible to calculate some smooth acceleration profile, for example using JMT technique.

### Unfortunately that did not work ###

To get stable acceleration profile it's critical to have good estimation of the initial acceleration __a0__. To get it I approximate the waypoints from the previous iteration with 2-nd degree polynomial and take the second member. (I also tried approximating with 3 degree polynomial and considering jerk, but results are the same)

Let's say real initial acceleration is 1 m/s2, but our estimation based on waypoint analysis returned 1.05. Looking on the total graph where 3 seconds planning horizon generate 150 time ticks you will not notice the difference. The problem is that the graph smoothly continues the previous path and first generated points will have __a__ close to 1.05. Because the planning algorithm is working pretty fast, the simulator uses only 2-3 waypoints and then calls my code again asking for the next path. That means __all__ waypoints really used by the simulator are generated based on the wrong acceleration and the car  will accelerate a bit. In the next iteration the process repeats. Having no chance to drive the whole provided path and only considering couple of first points that smoothly continue the wrong estimation the algorithm will accelerate the car indefinitely.

This problem arises only because the algorithm is intentionally working with very limited set of input data and has to reverse engineer the acceleration form the waypoints. Real cars have precise IMU sensors that measure acceleration directly and do not accumulate mathematical errors. I am using such sensors in my Arduino projects; they are cheap, precise and reliable.   
   
After several tries to solve the problem in a "right" way I decided to ignore the initial acceleration and generate profile only based on the desired velocity delta and planning horizon time. Theoretically that could lead to unnecessary jerk, but the simulator calls my code again and again many times per second with slightly different initial conditions and that makes the resulting trajectory smooth.      

## Trajectory planner ##

To generate smooth trajectories I am using spline interpolation. Advantage of that technique is that the resulting path is guaranteed to pass through the key points. Because the TK-Spline library requires point coordinates to not have loops along the x (time) axis, the coordinate system is first rotated at __-yaw__ angle. That makes car pointing exactly in x direction (TrajectoryHelper::buildTrajectory). After generating spline and waypoints their coordinates are rotated again at same angle in the opposite direction to get back into the world coordinate system (TrajectoryHelper::generatePath).

![Coordinate system](https://github.com/cazacov/CarND-Path-Planning-Project/blob/master/_img/coordinates.png?raw=true)    

### Smooth curves ###

A path always starts from the current car position and yaw and then goes through 4 key points on time scale from 0 to 1, where 1 corresponds to the whole planning horizon. The position of these key points is chosen depending on the situation:

![Splines](https://github.com/cazacov/CarND-Path-Planning-Project/blob/master/_img/splines.png?raw=true)

You can find that code in TrajectoryHelper::buildTrajectory function.  

### Validation ####

Every candidate trajectory is checked for violation of speed and acceleration limits. Calculating velocity and tangential acceleration is easy. To get the normal acceleration I need to estimate the path's curvature. Fortunately the [TK-Spline library](https://kluge.in-chemnitz.de/opensource/spline/) returns not only the value, but also first and second derivatives of the generated function. Curvature is then calculated with usual analysis formula:

![Curvature](https://wikimedia.org/api/rest_v1/media/math/render/svg/c92d581fd7b8a66308bde64d64dcd8d71cc66ce0)   

## Results ##

The car is able to drive at least two laps (10 miles) without incident and without exceeding speed, acceleration and jerk limits. The mean speed is about 45 MPH, it took almost exactly 8 minutes to drive 6 miles. Most of the time car drives at speed 48 MPH but sometimes decelerates to avoid collision.

## Possible Improvements

Currently the trajectory planner considers only staying in the current lane or changing one lane to the left or right. It does not evaluate trajectories that change lanes 1->3 or 3->1. For example in the following situation it's "blocked" and defensively decides to reduce speed, instead of trying to go in the leftmost lane:

![Blocked in the rightmost lane](https://github.com/cazacov/CarND-Path-Planning-Project/blob/master/_img/blocked.jpg?raw=true)
    

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

