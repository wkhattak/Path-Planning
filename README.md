# Project: Path Planning
## Overview
This project is about using a path planner to generate safe/optimal trajectory for navigating a car through a track that is shared by other cars that accelerate/decelerate  & change lanes.

## How Does It Work?

The path planner works on the basis of generating a set of waypoints that results in a smooth path, both for staying in the same lane and changing lanes. 

In a broader sense, the path planning involves the input of other modules as well as depicted in the below image:

![Path Planning](./images/intro.png)

Particularly, the path planning problem is concerned with the modules highlighted in the above image.  

**Note:** This particular implementation of the project is a simplified version of the path planner. A full blown implementation (in progress) can be viewed [here](https://github.com/wkhattak/Path-Planning-v2-Work-In-Progress).

The current implementation of the path planner can be summarised as below:

1. Find if ego car (autonomous car) is close to any non-ego car.
2. Take action e.g. slow down to keep following or change lane if possible.
3. Generate trajectory either for changing lane or staying in the same lane.

## Pertinent Information

The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

### Waypoints
The path planner should output a list of x and y global map coordinates. Each pair of x and y coordinates is a point, and all of the points together form a trajectory. Every 20 ms the car moves to the next point on the list. The car's new rotation becomes the line between the previous waypoint and the car's new location.

### Velocity
The velocity of the car depends on the spacing of the points. Because the car moves to a new waypoint every 20ms, the larger the spacing between points, the faster the car will travel. The speed goal is to have the car traveling at (but not above) the 50 MPH speed limit as often as possible. But there will be times when traffic gets in the way.

### Using Previous Path Points
Using information from the previous path ensures that there is a smooth transition from cycle to cycle. But the more waypoints we use from the previous path, the less the new path will reflect dynamic changes in the environment.

Ideally, we might only use a few waypoints from the previous path and then generate the rest of the new path based on new data from the car's sensor fusion information.

### Timing
The simulator runs a cycle every 20 ms (50 frames per second), but the C++ path planner will provide a new path at least one 20 ms cycle behind. The simulator will simply keep progressing down its last given path while it waits for a new generated path.

This means that using previous path data becomes even more important when higher latency is involved. Imagine, for instance, that there is a 500ms delay in sending a new path to the simulator. As long as the new path incorporates a sufficient length of the previous path, the transition will still be smooth.

A concern, though, is how accurately we can predict other traffic 1-2 seconds into the future. An advantage of newly generated paths is that they take into account the most up-to-date state of other traffic.

### Highway Map
Inside `data/highway_map.csv` there is a list of waypoints that go all the way around the track. The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow diving line in the centre of the highway.

The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.

The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

### Waypoint Data
Each waypoint has an (x,y) global map position, and a Frenet s value and Frenet d unit normal vector (split up into the x component, and the y component).

The s value is the distance along the direction of the road. The first waypoint has an s value of 0 because it is the starting point.

The d vector has a magnitude of 1 and points perpendicular to the road in the direction of the right-hand side of the road. The d vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's (x,y) coordinates with the d vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow diving line) is 2 m from the waypoint.

If you would like to be in the middle lane, add the waypoint's coordinates to the d vector multiplied by 6 = (2+4), since the centre of the middle lane is 4 m from the centre of the left lane, which is itself 2 m from the double-yellow diving line and the waypoints.

### Sensor Fusion
It's important that the car doesn't crash into any of the other vehicles on the road, all of which are moving at different speeds around the speed limit and can change lanes.

The sensor_fusion variable contains all the information about the cars on the right-hand side of the road.

The data format for each car is: `[id, x, y, vx, vy, s, d]`. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.

The vx, vy values can be useful for predicting where the cars will be in the future. For instance, if you were to assume that the tracked car kept moving along the road, then its future predicted Frenet s value will be its current s value plus its (transformed) total velocity (m/s) multiplied by the time elapsed into the future (s).

### Changing Lanes
Any time the ego vehicle approaches a car in front of it that is moving slower than the speed limit, the ego vehicle should consider changing lanes.

The car should only change lanes if such a change would be safe, and also if the lane change would help it move through the flow of traffic better.

For safety, a lane change path should optimise the distance away from other traffic. For comfort, a lane change path should also result in low acceleration and jerk.

### Data
Here is the data provided from the Simulator to the C++ Program

**Main car's localisation data (No Noise):**

[x] The car's x position in map coordinates

[y] The car's y position in map coordinates

[s] The car's s position in frenet coordinates

[d] The car's d position in frenet coordinates

[yaw] The car's yaw angle in the map

[speed] The car's speed in MPH

**Previous path data given to the planner:**

[previous_path_x] The previous list of x points previously given to the simulator

[previous_path_y] The previous list of y points previously given to the simulator

**Previous path's end s and d values:** 

[end_path_s] The previous list's last point's frenet s value

[end_path_d] The previous list's last point's frenet d value

**Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise):**

[sensor_fusion] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Rubric Points

### The code compiles correctly.
Yes, as shown by the following image:

![Code Compilation](./images/compilation.png)

### The car is able to drive at least 4.32 miles without incident.
Yes, below screenshot shows that the car was able to travel at least `13.54` miles without any incident.

![Max Mileage](./images/4.png)

### The car drives according to the speed limit.

Yes, as shown by the image above.

### Max Acceleration and Jerk are not Exceeded.

Yes, as shown by the image above.

### Car does not have collisions.

Yes, as shown by the image above.

### The car stays in its lane, except for the time between changing lanes.

Yes, as shown by the image above.

### The car is able to change lanes.
Yes, as shown by the images below.

![Lane Change Left](./images/1.png)

![Lane Change Right](./images/2.png)

### Reflection

The path planner is closely based on the project walkthrough video. An attempt was made to come up with an implementation that is based on the lesson map i.e. Predict non-ego cars' future location >>> Use behaviour planner to generate future states based on finite state machine >>> Choose best future state based on cost functions >>> Generate target trajectory via JMT (jerk minimising trajectory) generator. However, that implementation requires more time than originally thought, hence a decision was made to fork the implementation as a future project. That project (work in progress) can be viewed [here](https://github.com/wkhattak/Path-Planning-v2-Work-In-Progress).

The current project is a simplified version of the above project based on the following steps:

### 1. (Prediction) Find if the ego car is close to any non-ego car [Code Block: lines 275 - 312](https://github.com/wkhattak/Path-Planning/blob/master/src/main.cpp#L275).

Here the *sensor fusion* data is used to predict the future location of all non-ego cars. The aim is to find the nearest car that will be in front of us in the same lane. For the nearest non-ego car, both the speed of the non-ego car & the distance of the non-ego car are recorded. 

### 2. (Behaviour Planning) Find out which action should be taken [Code Block: lines 315 - 376](https://github.com/wkhattak/Path-Planning/blob/master/src/main.cpp#L315).

Based on the predictions, it is decided whether the ego car should stay in the lane or change lane. the logic goes as follows:

1. If close to the non-ego car in the same lane, slow down gradually but if too close then slow down quicker.
2. If close to the non-ego car in the same lane, check if a lane change is possible by calling `canChangeLane()`. If possible then set the `lane` to the required lane else stay in the same lane.
3. If no non-ego car within safe driving distance, accelerate gradually if below the speed limit.

### 3. (Trajectory Generation) Generate trajectory either for changing lane or staying in the same lane [Code Block: lines 379 - 516](https://github.com/wkhattak/Path-Planning/blob/master/src/main.cpp#L379).

1. Generate/pickup at least 2 previous unused waypoints if `prev_path_size` is less than 3, else pickup 3 points.
2. Add 3 more waypoints that are `30 metres` apart.
3. Use the [spline](http://kluge.in-chemnitz.de/opensource/spline/) library & assign the above generated waypoints as anchor points.
4. Use spline to generate `y` of the target `xy` based on known `x` which is the safe driving distance ahed (30 metres).
5. Find Euclidean distance from ego car to the target `xy`.
6. Find no. of road sections `N` by diving the *Euclidean distance* by the product of *velocity* & *delta t*. In our case, the *delta t* is `0.02` seconds as the simulator visits each waypoint ever 0.02 seconds. Basically the formula used is `target_distance = N*0.02*veloctiy` OR `N = target_distance/(0.02*veloctiy)`. N is the total no. of road sections from current location to safe driving distance ahead when travelling at `velocity`. Now by dividing the `target_distance` with `N` we get the optimum increment (in x-axis) between each waypoint while remaining below the threshold values of 10 m/s^2 (acceleration) and of 10 m/s^3 (jerk). By adding this `x_increment` to the current x position (0), we get the first `x`. Using spline again we can find the corresponding `y`. We only add that many points that were utilised in the previous cycle `50-prev_path_size`.
7. Each of the previously non-utilised & newly generated points are then added to the `next_x_vals` & `next_y_vals` collection for simulator's consumption.

### Shortcomings

1. No target trajectory generation for each possible future state.
2. No cost function used to decide between *lane change* & * keep lane* states.
3. Incorporate a controller such as PID or MPC that follows the Path Planner's output path. Note that since the output path contains not only desired location information but also the car's desired speed as varying spaced points. One idea is to extract the desired speed from the path and then feed that into the controller. Another idea is if working with an MPC is to change the cost function so instead of evaluating cost relative to how close you are to a path, instead evaluate by how close the car is to one of the associating points of the path's output.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


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

## Usage

Follow the build instructions above. Once the program is running, start the simulator. You should see a *connected!!!* message upon successful connection between the simulator and the c++ program. Hit the *Start button*. 

## Directory Structure

* **data:** Directory containing a list of waypoints that go all the way around the track
* **images:** Directory containing writeup images
* **src:** Directory containing c++ source files
* **CMakeLists.txt:** File containing compilation instructions
* **README.md:** Project readme file
* **install-mac.sh:** Script for installing uWebSockets on Macintosh
* **install-ubuntu.sh:** Script for installing uWebSockets on Ubuntu

## License

The content of this project is licensed under the [Creative Commons Attribution 3.0 license](https://creativecommons.org/licenses/by/3.0/us/deed.en_US).
