# Model Predictive Control
Self-Driving Car Engineer Nanodegree Program

---


## Project Overview

In this project, the model predictive control technique is implemented to drive the car around the track in the Udacity simulator. The simulator provides the car's position, direction, and reference track trajectory. 

[Video of the whole track](https://www.youtube.com/watch?v=fOTDIdAGvVc)


<p align="center">
  <img width="370" height="300" src="./images/simulation1.gif">
  <img width="370" height="300" src="./images/simulation2.gif">
</p>



### The Vehicle Model

The vehicle modle in this project is kinematic model, which has 4 elements in each states: `x` position, `y` position, heading direction `psi`, and velocity `v`. Here are the equations that show how state changes over time based on the previous state and current actuator inputs. 

```
  x[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
  y[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
  psi[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
  v[t] = v[t-1] + a[t-1] * dt
  cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
  epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
```

`Lf` in the equation of `psi` is the distatnce between the center of mass of the vhicle and its front axle. `delta` and `a` are actuators, which represent angle difference bewteen car direction and direction of reference track trajectory and acceleration, respectively. 
`cte` is the cross-track error. `epsi` is the orientation error.


### MPC Preprocessing

In the MPC preprocessing, all waypoints provided by the simulator are transformed to the vehicle's coordinate system. Then a reference track trajectory is built based on the polynomial fitting of transformed waypoints. The inital input of each prediction step is

```
x0 = 0
y0 = 0
psi0 = 0
v0 = current velocity
cte0 = coeffs[0]
epsi0 = -atan(coeffs[1])
```
`coeffs` is the coefficient array of fitted polynomial equation.


### Model Predictive Control with Latency

The latency of 100 ms is introduced in the project to simulate the actuation command delay in the real world. This means that the control signal received by the vehicle is actually the one which is sent 100ms ago. Therefore, we can predict the state after 100ms by using the vehicle model equations and use it as the input into the solver function. The actuation result from the solver function can be taken as the predicted control signal for the vehile after 100ms. 

```
x0 = v * cos(psi0) * latency_time;
y0 = v * sin(psi0) * latency_time;
psi0 = v * delta / Lf * latency_time;
v = v + a * latency_time;
cte0 = cte0 + v * sin(epsi0) * latency_time;
epsi0 = epsi0 + v * delta / Lf * latency_time;
```

### Timestep Length and Reference Velocity

The time `T = N * dt` is the prediction horizon. The length of `T` depends on the speed of the vehicle in the simulator. If the speed `v` of the car is very fast, then we need to have a relatively small `T`, since a very large `T` can make the predicted track trajectory (`T*v`) longer than the input reference track trajectory, which can results in bad control signals. 
Here I set `N = 10` and `dt = 0.11s`. 

The reference velocity is in the range of [25mps, 55mps], and it is determined by the curvature of the vehicle's position in the track. In the program, the curvature sum of four points which are car's current and closest future positions are calculated. Then the curvature sum is compared with a minimum and a maximum curvature sum value to determine the value of reference velocity.

If the curvature sum is smaller than the given minimum value `curvature_min`, the reference velocity can be set to its maximum value `ref_v_max`.
```
if (curvature_sum < curvature_min) {
  ref_v = ref_v_max;
} 
```

If the curvature sum is larger than the given maximum value `curvature_max`, the reference velocity can be set to its minimum value `ref_v_min`.
```
else if (curvature_sum > curvature_max) {
  ref_v = ref_v_min;
}
```

Here is the equation of calculating the reference velocity when it is between the given minimum and maximum value:
``` 
else {
  ref_v = ref_v_max - (ref_v_max - ref_v_min) * exp(15 * (curvature_sum - curvature_max));
}
```
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
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.


## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


