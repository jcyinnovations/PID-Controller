# Term 2 Project 4: PID Controller
Self-Driving Car Engineer Nanodegree Program

---

## Notes
This commit demonstrates the best results of training the controller to-date. The control is smooth and stable and has been tested stable with speeds up to 65 mph. Control is somewhat sloppy in some turns notably the third turn. Manual tuning of the parameters only served to increase the instability of the controller; likely indicating that some control over throttle would assist with turning. The throttle is switched between close to half and zero (no throttle) to maintain the set speed for running.

The control parameters were derived from using running the built-in Twiddle on the controller. Using Twiddle is triggered by passing the `PIDPhase` to the Init() function. `PIDPhase::TWIDDLE` starts Twiddle directly whereas `PIDPhase::RAMP` sets the car to a fixed speed before starting Twiddle.

To start each Twiddle loop, the simulator is reset of its starting position and the car is accelerated to a training speed of 40 mph using a fixed Kp=0.15. Twiddle is then run for 400 iterations (N) and the error used to update the parameter before the next loop. The choice of Kp and training speed for the car as well as N were determined through experimentation. It was hard to reach higher speeds without putting the car into the ditch before starting Twiddle and at 40mph, N=400 was just enough to get the car around the first turn. This allowed the training error to reflect any oscillations that occurred due to turning.

The increments of the parameters Kp, Ki and Kd were also lowered from 1.0 to 0.5, 0.25 and 0.25 respectively as this seemed to help training settle into a solution.


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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

