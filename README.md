# CarND-Controls-MPC-Project
Self-Driving Car Engineer Nanodegree Program

This is my 5th project of Udacity SDCNanodegree program term 2, which aims to implement Model Predictive Control and to tune the hyperparameters in C++ to maneuver the vehicle around the lake race track. The simulator will not provide the crosstrack error(CTE) and the program should calculate it automatically. Besides, there is a 100 millisecond latency between actuaction commands on the top connection latency. So the MPC controller should compensate it.

Therefore, my project should have a solution robust to 100ms latency as in real-world situation, and use IPOPT and CPPAD libraries to calculate an optimal trajectory and its related actuation commands, aiming to minimize the cost function of a 3-degree polynomial fit to the reference waypoints. The MPC optimizer only considers a short duration of waypoints before vehicle.

[//]: # (Image References)
[image1]: ./MPCSnapshot.png

---
# Simulator platform and Websocket Data 

Here is a snapshot of vehicle working situation. The yellow line is a polynomial fitted to reference waypoints, and green line represents the predicted coordinates of the MPC trajectory. A singular unity unit in the simulator is equivalent to 1 meter.
![alt text][image1]

The udacity-provided simulator communicates telemetry waypoints data via websocket, by sending steering and acceleration commands back to the simulator. Particularly, the JSON oject sends the data back from simulator server through websocket, which means that the simulator sends global coordinates of waypoints(ptsx, ptsy), the vehicle orientation(in terms of psi and psi_unity), the global coodinates of vehicle(x, y), the steering angle (steering_angle), the acceleration(throttle) and the vehicle speed(speed). And the detailed data fields are as follows:

- **Array ptsx** - The global x positions of the waypoints.
- **Array ptsy** - The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
- **float psi** - The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions.
- **float psi_unity** - The orientation of the vehicle in radians. This is an orientation commonly used in navigation.
- **float x** - The global x position of the vehicle.
- **float y** - The global y position of the vehicle.
- **float steering_angle** - The current steering angle in radians.
- **float throttle** - The current throttle value [-1, 1].
- **float speed** - The current velocity in mph.

---
# Implementation Details

The MPC process includes steps to accomplish this project as follows:
 - 1.Transforms waypoints from global world coordinates to vehicle coordinates 
 - 2.Use transformed waypoints to fit a 3-degree polynomial and get coeffs to calculate initial cte and epsi.
 - 3.Assume a latency of 100ms(0.1s)and predict the state using motion update equations of MPC model. 
 - 4.Send the predicted latent state and coeffs to the MPC controller(Optimizer, also called Solver). 
 - 5.Use the steer_value and throttle_value from MPC solution and send them to the simulator.
 - 6.Draw the MPC predicted trajectory and reference trajectory.
 - 7.Tune hyperparameters N and dt for smooth driving.

## I. The Model
Describe the model in detail. This includes the state, actuators and update equations.

### 1. Steer/Acceleration kinematic  model
From the lessons I know there're two kinds of vehicle dynamics models, describing how the vehicle moves: kinematic models and dynamic models. Thereinto, dynamic models aim to embody the actual vehicle dynamics as closely as possible. And kinematic models are their simplification, ignoring many elements like gravity, tire forces and mass. This simplification reduces the accuracy of models, but also makes them more tractable. At low and moderate speed, kinematic models often approximate the actual vehicle dynamics. 

In kinematic model, there are three common actuators: steering wheel, throttle and break pedals. For now I will consider throttle and break pedals as one actuator, named acceleration. Negative value of acceleration means braking and positive value means accelerating. Therefore I have two control inputs in actuator vector: steering angle and acceleration.

Model Predict Control(MPC) considers vehicle following a trajectory as an optimization problem, and the solution of optimization problem is the optimal trajectory. It involves simulating different actuactor inputs, predicting the result trajectory and selecting that trajectory with a minimum cost.

### 2. State vector
The state vector has 6 variables as follows, regard of 2 error elements like cross track error(cte) and orientation error(epsi).

State|Description|Units|Constraints
-------|----------|-----|----
x|vehicle position x| m|[-1.0e19,1.0e19]
y|vehicle position y| m|[-1.0e19,1.0e19]
psi|vehicle heading orientation|radians|[-1.0,1.0]
v|vehicle speed| mph|[30,100]
cte|the difference between reference path and predicted MPC trajectory|m|[-1.0e19,1.0e19]
epsi|the difference between reference path orientation and predicted MPC trajectory orientation|radians|[-1.0e19,1.0e19]


### 3. Actuator vector
The actuator vector has 2 variables as follows:

Actuator|Description|Units|Constraints
------|-------|----|----
delta|steering angle/value to drive|randians|[-25,25]degree equals to [-0.436332,0.436332]
a|throttle/brake|pow(mph,2)|[-1,1]

### 4. Update equations

In steer/throttle kinematic model, how the state changes over time is based on the previous state and current acuator inputs as follows.
```
x[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t]/Lf * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t]/Lf * delta[t] *dt
```
You can find the related original code snippet to calculate the state changes in my MPC.cpp, where fg[0] stores the final cost of optimization process, and fg[1:] store all the variable changes.
```
fg[1+x_start+t] = x1-(x0 + v0*CppAD::cos(psi0)*dt);
fg[1+y_start+t] = y1-(y0 + v0*CppAD::sin(psi0)*dt);
fg[1+psi_start+t] = psi1 - (psi0 + v0/Lf*delta0*dt);
fg[1+v_start+t] = v1 - (v0+a0*dt);
fg[1+cte_start+t] = cte1 - ((f0 - y0) + (v0/Lf*CppAD::sin(epsi0)*dt));
fg[1+epsi_start+t] = epsi1 - ((psi0-psides0) + v0/Lf*delta0*dt);
```
Note that there is a **sign invesion problem here about psi and epsi update equations**. And I will discuss it in the next subsection "about steer value".

### 5. About steer value

As the [lesson 2- Tips and Tricks of MPC project](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/929d26d4-f93c-429c-8bd3-728154e44f82) said: "Note if delta is positive we rotate counter-clockwise, or turn left. In the simulator however, a positive value implies a right turn and a negative value implies a left turn."

Refering to the forum[here is some advice about steering values](https://discussions.udacity.com/t/here-is-some-advice-about-steering-values/276487/5) which also analyzed this detail, the counter-clockwise angles on the simulator steering command scale are negative, while counter-clockwise angles in the vehicle model are positive. In other words, on the simulator steering value scale, "left" means negative, while in the vehicle model, "left" means positive. So when the optimizer outputs a positive delta angle, that means the vehicle should steer left. But in order to tell the simulator to steer right, I need to give it a negative value. So there are two alternative ways to handle this sign inversion proble:

 - method 1.change the psi update equation to  $psi[t+1] = psi[t] - v[t] / Lf * delta[t] * dt$, and correspondingly, the epsi update equation will alos be change as $epsi[t+1] = psi[t] - psides[t] - v[t]/Lf * delta[t] *dt$.
 - method 2.keep the psi and epsi update equations unchanged, and just multiply the optimizer output(steering value) by -1.0 before sending back to the simulator. 

In practice, if I didn't invert the steer_value sign, the vehicle will have a zigzag trajectory in the very begining and have a crush out of track unfortunately. So I tried the possible ways above to invert the steer value's sign. Both of these two ways will work, but the method 1 works better and more smoothly with less errors. And I haven't figure out why so far. 

**Here is a summary about steering value in this project**: 

The simulator takes as input values in [-1,1], where -1 equals to 25 degrees to the left and 1 equals to 25 degrees to the right. The simulator returns values in [-0.436332, 0.436332], where -0.436332 is 25 degrees to the left and 0.436332 is 25 degrees to the right. Before I pass the steer_value from MPC optimizer to simulator, I should invert the steer value' sign first. Both method 1 and method 2 work, but method 1 works better in my practice.

### 6. Cost function
Define the MPC cost function with components of reference state, actuactor, etc. In the MPC.cpp of my project, the cost function consists of 3 parts as follows:
```
    ///* all the costs are stored in the first element of fg, including initial one and following ones
    fg[0] = 0;
    ///* define the cost function related the reference state and anything beneficial
    ///* about the reference state and set different empirical weights
    for (size_t t = 0; t < N; t++) {
      fg[0] += 3000 * CppAD::pow(vars[cte_start + t]- ref_cte, 2);
      fg[0] += 2800 * CppAD::pow(vars[epsi_start + t]- ref_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v - ref_v, 2);
    }    
    ///* about the actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 20CppAD::pow(vars[a_start + t], 2);
    }
    ///* about the differences between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += 100 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 10 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }
```
The different weights are chosen empirically to give the relative cost function component proper penalty, to make the vehicle not behave erratically and drive more smoothly.

## II. Timestep Length and Elapsed Duration(N & dt) 

N is the timestep length and dt is the elapsed duration between timesteps. Both of them define the "look ahead time" of the MPC. In this project I choose these two hyperparameters manually. 

As a summary, small dt(dt = 0.0125, 0.05) reduces the distance between two points in the predicted trajectory but requires more processing time, which may lead to vehicle oscillation, because the steering and throttle commands are sent too late. While N defines the point number of MPC predicted trajectory for drawing a green line. Small N( N= 6, 8) will produce shorter trajectory, which might make vehicle decrease the speed too late during turning. 

Therefore, the dt and N should be both large enough for driving path to stable. And my final chosen values are N = 10 and dt = 0.1, which actually build a one-second "look ahead duration" before vehicle to decide a correct trajectory. And these values made my vehicle drive very smoothly and successfully on the lake track, demonstrating a good trade-off between processing time and "lookahead time" for vehicle.

## III. Polynomial Fitting and MPC Preprocessing 

In my opinion, the MPC preprocessing for state data includes 3 steps, and I will discuss the first and second steps here, leaving the step 3 in the next section for analysis.

 - step 1.Transforms waypoints from global world coordinates to vehicle coordinates 
 - step 2.Use transformed waypoints to fit a 3-degree polynomial and get coeffs to calculate initial cte and epsi.
 - step 3.Assume a latency of 100ms(0.1s)and predict the state using motion update equations of MPC model. 
 
In step 1, the simulator provides telemetry data including global coordinates of waypoints and global coordinates of vehicle position. To draw and predict the MPC trajectory, the vehicle coordinates of waypoints are needed. Thus according to formula $$GlobalPoints = TransfromMatrix * VehiclePoints + VehicleInGlobalPoints$$, I can transform waypoints from global world coordinates to vehicle coordinates, using $$VehiclePoints = TransformMatrix.inverse()*(GlobalPoints - VehicleInGlobalPoints)$$. Notice the rotation angle'orientation is oppsite with vehicle's orientation.

Here is a code snippet in my main.cpp.
```
    ///*the vehicle coordinates of waypoints
    Eigen::VectorXd waypoints_x_v(waypoints_x.size());
    Eigen::VectorXd waypoints_y_v(waypoints_y.size());
          
    ///*transform waypoints from global world coordinates to vehicle coordinates
    for (size_t i = 0; i < waypoints_x.size(); i++) {
        double dx = waypoints_x[i] - x;
        double dy = waypoints_y[i] - y;
        waypoints_x_v[i] = dx * cos(-psi) - dy * sin(-psi);
        waypoints_y_v[i] = dx * sin(-psi) + dy * cos(-psi);
    }
```
In step 2, after transforming waypoints from global world coordinates to vehicle coordinates, I can assume the vehicle position in vehicle coordinates is (0,0) and orientation psi = (0.0), for simiplifying the future computations like polynomial fit, original cte and epsi calculation and latency estimation. In other world, the vehicle is the origin of vehicle coordinates. Here is the codeline for polynomial fitting in my main.cpp:
```
   ///* fits a 3 order polynomial to the waypoints x and y
   auto coeffs = polyfit(waypoints_x_v,waypoints_y_v,3);
```
Then since the vehicle position is assumed as(0,0) with psi = 0.0, the cte = polyeval(coeffs,vehicle_x)-vehicle_y and epsi = psi - atan(coeffs[1]) with derivative of the polyfit equals to atan(), where when vehicle_x = 0.0 the other higher orders are 0, leaves only coeffs[1]. Here is the codeline for cte and epsi calculation.
```
  ///* calculate the cross track error and orientation error
  double cte = polyeval(coeffs, 0.0) - 0.0;
  double epsi = psi - atan(coeffs[1]);
```
Then the calculated cte and epsi can be used in the following step 3: latency estimation.


## IV. Model Preditive Control with Latency
The model predictive control is implemented to handle a 100 milisecond latency. And I will provides the details on how they deal with latency with 2 parts. 

- part 1. In my main.cpp, to compensate this latency of 100ms, the 6-element state vector should be predicted by my steer/acceleration kinematic model 100ms ahead before MPC optimizer/solver is called. According to the forum [how to incorporate latency into the model](https://discussions.udacity.com/t/how-to-incorporate-latency-into-the-model/257391/35), note the sign of steer value should be inverted.

```
  ///* Add latency to state with steer_value and throttle_value
  const double Lf = 2.67;
  const double latency = 0.1;
  
  double pred_x = 0.0 + v*cos(0.0)*latency; 
  double pred_y = 0.0 + v*sin(0.0)*latency; 
  double pred_psi = 0.0 + v*(-steer_value)/Lf*latency;
  double pred_v = v + throttle_value*latency;
  double pred_cte = cte + v*sin(epsi)*latency;
  double pred_epsi = epsi + v*(-steer_value)/Lf*latency;

  state<<pred_x,pred_y,pred_psi,pred_v,pred_cte,pred_epsi;
```
- part 2, in my MPC.cpp, in original state and actuator update equation of kinematic model, the state at t+1 depends on previous state and previous actuation commands at t, due to a delay of 100ms. Here is the codeline:
```
  ///* only consider the actuations at t, use previous actuations to accout for latency
  AD<double> delta0 = vars[delta_start+t-1]; // t = 1,...,N
  AD<double> a0 = vars[a_start+t-1];
```

---	
# Performances
My MPC is implemented on Ubuntu 14.04 LTS,with a Dell Inspiraon 14 7000(Intel Core i7-7500U CPU@2.7GHz*4 with 8GB RAM).  And the performance is demonstrated in **./MPCFinal.mp4**. 

Please refer to it.

---

# Dependencies

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


# Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

# Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.


# Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.