# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Rubric Criteria

### Compilation
#### "Your code should compile."

The code compiles without any errors with cmake & make.

### Implementation
#### "The Model"

The model used is a Kinematic model (a simplified dynamic model) that ignores tire forces, gravity and mass, as we
learned in class. The simplification reduces the accuracy of the models, but it also makes them more tractable.
The model looks as follows:

![MODEL](https://github.com/SSJRicer/SDCE-T2-P5-MPC/blob/master/Images/Model.png?raw=true)

State vector:
* `x`   : Car's x position (in the car's space).
* `y`   : Car's y position (in the car's space).
* `psi` : Car's heading.
* `v`   : Car's velocity.
* `cte` : Car's cross track error (from the middle of the lane).
* `epsi`: Car's heading error.

Actuator/Constraints vector:
* `delta`: Steering angle.
* `a`    : Car's throttle.

Where L<sub>f</sub> (given by Udacity) is the distance from the car's center of mass to its front wheels' axis.

The model takes the initial state of the car at each iteration of movement and sends it to the
solver which, using the IPOPT and CPPAD libraries, minimizes the cost function (J) in order to find the right
acceleration and steering angle so our car will drive in the middle of the lane.

The cost function consists of the following factors (all of which were learned in class):
* Squared sum of CTE and EPSI (our references were 0) and (v - v<sub>ref</sub>) (reference = 100 so our car drives faster).
* Squared sum of actuators (penalizing their use) - impacts their magnitude.
* Squared sum of actuators change between each step - adds temporal smoothness.

The weights of each of the squared sums were chosen through intuitive trial & error.

#### "Timestep Length & Elapsed Duration (N & dt)"

The Prediction Horizon (T = N * dt) is chosen first (as suggested in class) since we need to first
figure out how many seconds into the future we want to predict.
 
* **Note:** Technically it will be better if the overall time T was chosen dynamically, proportional 
to the velocity, but given our choice of parameters and reference velocity, anywhere between 0.75-1.5 seconds
should do the job.

As for the timestep length (N) and elapsed duration of each step (dt) - I went with a trial & error 
approach based on the initial given values of 10/0.1 and discovered the following:

|   N   |   dt   |  Conclusion  |
|:-----:|:------:|:------------:|
|   10  |   0.1  | Initial given values - works quite well                         |
|   25  |   0.01 | Slower drive but smoother turns                                 |
|   40  |   0.5  | Can't converge (dt too high)                                    |
|   40  |   0.1  | Better start from previous choice, but still too many steps     |
|   20  |   0.1  | Too many steps - loses control on turns                         |
|   **15**  |   **0.05** | **Final & best result - smooth turns and steady drive** |

#### "Polynomial Fitting & MPC Preprocessing"

I preprocessed the waypoints given by the simulator to the car's coordinate system, thus making the 
origin point and heading angle zero - car as reference point (src/main.cpp - lines 57-65).

Afterwards I created a 3rd degree polynomial from the transformed waypoints in order to calculate the CTE and EPSI
in the model (src/main.cpp - lines 67-73).

#### "Model Predictive Control with Latency"

Handling a 100 millisecond latency was done in the initial state, which was taken based on the car's
coordinate system (initial state being (0, 0) with heading 0), and delayed by 100 millisecond (t+1 state)
and only then sending the state into the solver (src/main - lines 75-100).

### Simulation
#### "The vehicle must successfully drive a lap around the track."

The vehicle does indeed successfully drive a lap without rolling over any surfaces.

---

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

## Build with Docker-Compose
The docker-compose can run the project into a container
and exposes the port required by the simulator to run.

1. Clone this repo.
2. Build image: `docker-compose build`
3. Run Container: `docker-compose up`
4. On code changes repeat steps 2 and 3.

## Tips

1. The MPC is recommended to be tested on examples to see if implementation behaves as desired. One possible example
is the vehicle offset of a straight line (reference). If the MPC implementation is correct, it tracks the reference line after some timesteps(not too many).
2. The `lake_track_waypoints.csv` file has waypoints of the lake track. This could fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We have kept editor configuration files out of this repo to
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
agnostic as possible. We omitted IDE profiles to ensure
students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. Most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio and develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
