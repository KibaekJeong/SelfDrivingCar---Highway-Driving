# **Highway Driving**
---

 **Path_Planning_Project**

The goals / steps of this project are the following:

In main goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The car's localization sensor fusion data, and a sparse map list of waypoints around the highway is given as input data.
The goals / steps of this projects are following:
- Car should try to go as close as possible to the 50 MPH speed limit
- Pass slower traffic when possible
- Avoid hitting other cars at all cost
- Drive inside of the marked road lanes
- Able to make one complete loop around 6946m highway
- Should not make total acceleration over 10 m/s^2
- Jerk should not be greater than 10 m/s^3

[//]: # (Image References)

[image1]: ./output/output_img.png "output"
---

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

## Simulator.
You can download the Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).  

To run the simulator on Mac/Linux, first make the binary file executable with the following command:
```shell
sudo chmod u+x {simulator_file_name}
```

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Project Approach

In order to successfully navigate through the highway, first approach is to make the car be able to follow the lane. Using the sparse map list of the highway, way points are provided to car to follow. Car is designed to drive each points per 0.02 seconds. We can create an equation describing following car movement by: N * 0.02 * velocity = distance, where N is number of points. Also, velocity in following project needs to be converted to mph from m/s. Therefore, 0.447 has been multiplied to velocity for conversion. After getting way points and letting the car be able to drive at desired velocity, which is close to speed limit(50mph), we need to consider interaction with traffic.
There are two things that we need to do with traffic. First, self-driving car should not collide with another car. Second, car should pass another traffic when possible. Also, we should consider cars that change lane. In order to not collide with traffic, car is designed to decelerate when another vehicle is located ahead. Also, for better safety, when another car is located ahead and very close, it is designed to decelerate at a higher rate. When decelerated, it is designed to decelerate until cars velocity is slightly slower than the velocity of the car ahead. Next, when car is ahead, car changes lane to empty lane whenever possible. Car prioritize changing to left lane, as left most lane is overtaking lane. Lane is considered to be empty when there is no vehicle at anywhere 30m ahead and 25m behind the main car.

## Result
Car was able to navigate through the highway without incident for longer than 10 Miles. However, car sometimes hit other cars when other cars change lane, specifically when the car is located close to main car and changes lane. Below is the image taken out from the simulator.

![alt text][image1]


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
