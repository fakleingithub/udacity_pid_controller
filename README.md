# **CarND-Path-Planning-Project**
Self-Driving Car Engineer Nanodegree Program
   
---

[//]: # (Image References)

[gif1]: ./media/pid.gif "PID Controller"


## The PID procedure 

```cpp
	pid.diff_cte = cte - pid.prev_cte;
	pid.prev_cte = cte;

	pid.int_cte = pid.int_cte + cte;

	steer_value = - pid.Kp * cte - pid.Kd * pid.diff_cte - pid.Ki * pid.int_cte;
```


## Description of the effect each of the P, I, D components have.

`cte` (cross-track-error)
is the latteral distance between the vehicle and the reference-trajectory, used for proportional part of PID controller to minimize the distance to the reference-trajectory (car is overshooting when just using the proportional error correction)

`diff_cte` (differential cross-track-error)
is the difference of the current cross-track-error and the previous one, for lowering or avoiding the overshoot which occur when just using proportional correction (Kp * cte)

`int_cte` (integrated cross-track-error)
is the sum of all cross-track-errors ever observed during runtime, for correcting systematic bias

The car reacts as expected to the increase or decrease of each component.

## Description how the final hyperparameters were chosen.

Initial values were chosen for `Kp=0.2`, `Kd=3.0` and `Ki=0.004`.

To improve the hyperparameters while the car drives I implemented the Twiddle algorithm.
The algorithm sequentially updates the PID-parameters, by first trying to increase one parameter, when the error decreases, we keep the increased parameter and increase the adoption rate. If the error increases, we decrease the parameter and decrease the adoption rate. When the error then decreases, we keep the decreased parameter and increase the adoption rate. If not we only lower the adoption rate.

The initial adoption rate was 0.01 for `Kp`, 0.1 for `Kd` and 0.0001 for `Ki`

After approximately 30 minutes of training time the final parameters were `Kp=0.171378`, `Kd=2.70802` and `Ki=0.00388669`

with the adopiton rate decreased to 0.000523348 for `Kp`, 0.00523348 for `Kd` and 0.00000471013 for `Ki`

```cpp

if (twiddle_activated == true) {
      // activate twiddle algorithm for training the pid-parameters
    
      if ( stepcount >= stepsize ) {
        err = err + (pow(cte, 2.0));
      }
      if ( stepcount >= 2 * stepsize) {
        best_err = err / stepsize;
        stepcount = 0;
        runcount = runcount + 1;
        best_err_calculated = true;
      }
      if (best_err_calculated == true){
        if (runcount <= 1){
          p[i_pid] += dp[i_pid];
        }
        if (runcount >= 2) {
          if (err < best_err){
             best_err = err; 
             dp[i_pid] *= 1.1;
          } 
          else {
            if (error_greater_best_prev == false){
              p[i_pid] -= 2*dp[i_pid];
              error_greater_best_prev = true;
            } 
            else {
               p[i_pid] += dp[i_pid];
               dp[i_pid] *= 0.9;
               error_greater_best_prev = false;
            }
          }
          pid.Adapt(p[0], p[1], p[2]);
        }
        best_err_calculated = false;
      }
       if (runcount == 4){
            runcount = 0;
            i_pid = i_pid+1;
            if (i_pid == 3){
               i_pid = 0;
            }
       }
```

## The vehicle must successfully drive a lap around the track.

The vehicle drives successfully multiple rounds around the track.
Here you can see a part of the final result:

![pid controller][gif1]


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)


## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).


