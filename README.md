[car0_lidar_nis]: ./image/car0_lidar_nis.png "car0_lidar_nis"
[car0_radar_nis]: ./image/car0_radar_nis.png "car0_radar_nis"
[car0_px]: ./image/car0_px.png "car0_px"
[car0_py]: ./image/car0_py.png "car0_py"
[car0_vel]: ./image/car0_vel.png "car0_vel"
[car0_yaw]: ./image/car0_yaw.png "car0_yaw"
[car1_lidar_nis]: ./image/car1_lidar_nis.png "car1_lidar_nis"
[car1_radar_nis]: ./image/car1_radar_nis.png "car1_radar_nis"
[car1_px]: ./image/car1_px.png "car1_px"
[car1_py]: ./image/car1_py.png "car1_py"
[car1_vel]: ./image/car1_vel.png "car1_vel"
[car1_yaw]: ./image/car1_yaw.png "car1_yaw"
[car2_lidar_nis]: ./image/car2_lidar_nis.png "car2_lidar_nis"
[car2_radar_nis]: ./image/car2_radar_nis.png "car2_radar_nis"
[car2_px]: ./image/car2_px.png "car2_px"
[car2_py]: ./image/car2_py.png "car2_py"
[car2_vel]: ./image/car2_vel.png "car2_vel"
[car2_yaw]: ./image/car2_yaw.png "car2_yaw"

# SFND_Unscented_Kalman_Filter

## Statistic result graph
I show the statistic result for each car below, it looks like that the estimated yaw is not good.

### Car 0
1. lidar nis:
   
   ![car0_lidar_nis]
2. radar nis:
   
   ![car0_radar_nis]
3. px:
   
   ![car0_px] 
4. py:
   
   ![car0_py]
5. vel:
   
   ![car0_vel]
6. yaw:
   
   ![car0_yaw]

### Car 1
1. lidar nis:
   
   ![car1_lidar_nis]
2. radar nis:
   
   ![car1_radar_nis]
3. px:
   
   ![car1_px]
4. py:
   
   ![car1_py]
5. vel:
   
   ![car1_vel]
6. yaw:
   
   ![car1_yaw]

### Car 2
1. lidar nis:
   
   ![car2_lidar_nis]
2. radar nis:
   
   ![car2_radar_nis]
3. px:
   
   ![car2_px]
4. py:
   
   ![car2_py]
5. vel:
   
   ![car2_vel]
6. yaw:
   
   ![car2_yaw]

Sensor Fusion UKF Highway Project Starter Code

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

In this project you will implement an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements. Passing the project requires obtaining RMSE values that are lower that the tolerance outlined in the project rubric. 

The main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./ukf_highway

Note that the programs that need to be written to accomplish the project are src/ukf.cpp, and src/ukf.h

The program main.cpp has already been filled out, but feel free to modify it.

<img src="media/ukf_highway.png" width="700" height="400" />

`main.cpp` is using `highway.h` to create a straight 3 lane highway environment with 3 traffic cars and the main ego car at the center. 
The viewer scene is centered around the ego car and the coordinate system is relative to the ego car as well. The ego car is green while the 
other traffic cars are blue. The traffic cars will be accelerating and altering their steering to change lanes. Each of the traffic car's has
it's own UKF object generated for it, and will update each indidual one during every time step. 

The red spheres above cars represent the (x,y) lidar detection and the purple lines show the radar measurements with the velocity magnitude along the detected angle. The Z axis is not taken into account for tracking, so you are only tracking along the X/Y axis.

---

## Other Important Dependencies
* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
 * PCL 1.2

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./ukf_highway`

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html) as much as possible.

## Generating Additional Data

This is optional!

If you'd like to generate your own radar and lidar modify the code in `highway.h` to alter the cars. Also check out `tools.cpp` to
change how measurements are taken, for instance lidar markers could be the (x,y) center of bounding boxes by scanning the PCD environment
and performing clustering. This is similar to what was done in Sensor Fusion Lidar Obstacle Detection.

## Project Instructions and Rubric

This information is only accessible by people who are already enrolled in Sensor Fusion. 
If you are enrolled, see the project page in the classroom
for instructions and the project rubric.

## Discussion link
1. [Can't seem to process and update measurements in final project](https://knowledge.udacity.com/questions/355044)
2. [Initialize P_](https://knowledge.udacity.com/questions/486035)
3. [ukf initilization with radar](https://knowledge.udacity.com/questions/487026)
4. [Radar Initialization and uncorrect RMSE estimation](https://knowledge.udacity.com/questions/396264)
5. [the car_2 prediction wrong, although other cars work](https://knowledge.udacity.com/questions/398805)
