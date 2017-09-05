# Korda's Model Predictive Controller Project for Udacity Self-driving Car Nanodegree

[YouTube Video](https://youtu.be/UY5qKKaF1ik)

[![alt text](https://img.youtube.com/vi/UY5qKKaF1ik/0.jpg)](https://youtu.be/UY5qKKaF1ik)

### This is my submission for the project described below. I implemented a Model Predictive Controller (MPC) for the steering and throttle of a simulated car given waypoint information from the simulator. [Here are the rubric points of the project](https://review.udacity.com/#!/rubrics/896/view)

### The main focus of the project was tuning the cost functions, time horizon, and time step to arrive at a stable controller that would make it around the track smoothly and in a timely manner. We also had to account for 100ms of actuator latency in simulator. This was meant to approximate real world latency in self-driving cars. I accomplished the latency corrections by calulating where the car would be after the 100ms and using that position and orientation to feed the controller state. This assured that the input that the controller commanded was an input appropriate for the vehicles position 100ms in the future.

### The model is constrained using the following functions with vehicle x, y (position), psi (heading), v (velocity), cte (cross-track error), and epsi (heading error):

```
// Model contraints: x
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 * dt / Lf);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
```
### Bounds were set for the actuators to prevent overdriving them:

```
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  
  // Acceleration/decceleration upper and lower limits.
  const double throttle_min = -1.0;
  const double throttle_max = 1.0;
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = throttle_min;
    vars_upperbound[i] = throttle_max;
```

### I began my search for the proper cost function weights at unity for all and no correction for latency, just so I could see what the baseline performance was. This led to unsurprisingly poor results illustrated in the video below. The controller starts out looking normal but very quickly develops huge instabilities. (see video below)

[YouTube Video](https://youtu.be/GbqSm-CET-8)

[![alt text](https://img.youtube.com/vi/GbqSm-CET-8/0.jpg)](https://youtu.be/GbqSm-CET-8)

### I then tried to add 100X cost to the orientation (psi) error, while leaving all other cost weights at unity. This helped a little and it almost looked like it was going to be stable but again instabilities grew after the first few seconds. (see video below)


[YouTube Video](https://youtu.be/GE71lUcRukY)

[![alt text](https://img.youtube.com/vi/GE71lUcRukY/0.jpg)](https://youtu.be/GE71lUcRukY)


### Then I tried bumping up the cost for using the steering actuator. This helped to smooth out the instabilities much longer but again it only bought me some time before it became unstable again. (see video below)


[YouTube Video](https://youtu.be/05LQ86IjgGA)

[![alt text](https://img.youtube.com/vi/05LQ86IjgGA/0.jpg)](https://youtu.be/05LQ86IjgGA)


### Now that I could see which weights in the cost funcitons were helping stabilize the system I implemented latency correciton and brought the cost weights back to unity to see how it affected the system. The latency correction was slightly better than raw unity in stayin on track for a few seconds but was still very poor. (see video below)


[YouTube Video](https://youtu.be/ZCOL-b-2aQk)

[![alt text](https://img.youtube.com/vi/ZCOL-b-2aQk/0.jpg)](https://youtu.be/ZCOL-b-2aQk)


### Now that I had seen all the effects of each weight and implemenation independently I spent a few hours tuning the cost weights until I arrived a stable system with the following weight and cost functions:

```
// adjustable weights for costs
    const double w_cte = 1;
    const double w_epsi = 100;
    const double w_v = 1;
    const double w_delta = 10000;
    const double w_a = 7;
    const double w_delta_smooth = 1000;
    const double w_a_smooth = 1;

    // The part of the cost based on the reference state.
    for (int t = 0; t < N; t++) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    
    // Minimize the use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + t], 2);
    }
    
    // Minimize the value gap between sequential actuations.
    for (int t = 0; t < N - 2; t++) {
      fg[0] += w_delta_smooth * CppAD::pow(vars[delta_start + t + 1]
                                         - vars[delta_start + t], 2);
      fg[0] += w_a_smooth * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

```

### As you can see in the video below the vehicle is stable and predicts a proper path for negotiating corners even at considerable speed (75 mph). 

[YouTube Video](https://youtu.be/UY5qKKaF1ik)

[![alt text](https://img.youtube.com/vi/UY5qKKaF1ik/0.jpg)](https://youtu.be/UY5qKKaF1ik)

### Now that the system was stable I played around with the number of time steps, N and the time between steps, dt. If N was too small the vehicle did not have enough sight into the future to see around conrners. Conversely, if N was too large there were some really strange curve fitting problems that led to unusable data. After playing I settled on N = 20 and dt = 0.05. this led to the best look ahead without looking too far ahead. I chose 0.05 because it was twice as fast as the latency of the system which I believed would provide enough continuity in the actuations. This choice of N and dt led to a stable system that tracked well and did not exhibit dicretization errors.

### I learned that there is a delicate balance when it comes to tuning an MPC. For this implementation I really had to tune up the cost of using the steering actuator to stabilize oscillations. The throttle control was not as critical. I did try to implement a cost function to reduce throttle during high steering angles but was not successfull in separating throttle control from steering control. This is one thing I would definitely like to implement in the future. 

#### Thanks to the Udacity content creators for making yet another fun and challenging project.

---

# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

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
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
       +  Some Mac users have experienced the following error:
       ```
       Listening to port 4567
       Connected!!!
       mpc(4561,0x7ffff1eed3c0) malloc: *** error for object 0x7f911e007600: incorrect checksum for freed object
       - object was probably modified after being freed.
       *** set a breakpoint in malloc_error_break to debug
       ```
       This error has been resolved by updrading ipopt with
       ```brew upgrade ipopt --with-openblas```
       per this [forum post](https://discussions.udacity.com/t/incorrect-checksum-for-freed-object/313433/19).
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/).
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `sudo bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

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

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
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

