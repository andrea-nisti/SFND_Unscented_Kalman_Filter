# SFND_Unscented_Kalman_Filter: Sensor Fusion UKF Highway Project

This is the final project for the Udacity Sendor Fusion nanodegree.

<img src="media/ukf_highway_tracked.gif" width="700" height="400" />

This project implements an Unscented Kalman Filter to estimate the state of multiple cars on a highway using noisy lidar and radar measurements.
The final solution was achieved by implementing an external library, called simpleukf, to perform the core computations, and then encapsulating it as a git submodule int the fusion::UKF class. 

Please check here: https://github.com/andrea-nisti/simple-ukf

## How to build and run

The main program can be built and ran by doing the following from the project top directory.

1. `git clone https://github.com/andrea-nisti/SFND_Unscented_Kalman_Filter.git && cd SFND_Unscented_Kalman_Filter`
2. `git submodule update --init --remote` to pull the external submodule
3. `mkdir build` to create the build directory
4. `cmake -S . -B build/ -D CMAKE_BUILD_TYPE=Debug` to configure the project for a debug build (put `Release` otherwise) 
5. `cmake --build ./build -j8` to build with 8 jobs

and finally, run the program with `./build/ukf_highway`


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
