## Udacity's Self-Driving Car Engineer Nanodegree Program
---
## PID Controller Project
---

The goal of the project is to implement PID controller in C++ and test/tune it using a simulator. General idea behind PID control is based on three terms (as per https://en.wikipedia.org/wiki/PID_controller):
* Proportional Controller: the motor current is set in proportion to the existing error.
* Integral term: increases action in relation not only to the error but also the time for which it has persisted.
* Derivative term: do not consider the error, but the rate of change of error, trying to bring this rate to zero. 

### Parameter Tuning
For the parameter tuning I have written Twiddle algorithm (https://www.youtube.com/watch?v=2uQ2BSzDvXs) yet its usage was limited due to coupling to the simulator. I have proceed tuning through trial and error in the following order:
* The proportional component has a strong effect on the output. its value was set to low, 0.07 (0.3 for throttle). High value can lead to overshoot and oscillations.
*  The derivative component calculates the future error based on the previous error and brings it into feedback system. It smooths effect from the proportional term. This term was set at 2.3 for steering, and 0.55 for throttle.
* The integral component sums errors over time. It reduces oscillations around the target value, but can introduce instability at the beginning when set too high. It has been choosen at 0.001.

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) - to connect with simulator
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`.