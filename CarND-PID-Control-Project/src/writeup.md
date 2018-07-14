# **PID Controller Project**

## Writeup

---

**Build a PID Controller Project**

The goals / steps of this project are the following:
- Implement PID controller in C++
- Tune the hyper-parameters of the controller to drive a virtual vehicle around the designated track

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/824/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/Angelenos/CarND-Term2/tree/master/CarND-PID-Control-Project/src)

### Implementation of Controller

#### 1. The PID procedure follows what was taught in the lessons. It's encouraged to be creative, particularly around hyperparameter tuning/optimization. However, the base algorithm should follow what's presented in the lessons.

Implementation can be found within line 9 to 31 from [PID.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-PID-Control-Project/src/PID.cpp) and line 4 to 53 from[PID.h](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-PID-Control-Project/src/PID.h).

Here as implemented in the quiz, the P, I and D errors are calculated as below:

- d_error = cte - p_error (**here p_error still holds the cte from previous timestamp**)
- p_error = cte
- i_error = i_error + cte

Here for debug purpose I add a bool parameter "debug" as a public member of the PID class. When it is "True" the member function PID::TotalError() will print the current P, I and D error every time it is called. It is by default to be false from the default constructor but I add another constructor so its value can be specified when it is defined.

### Reflection

#### 1. Describe the effect each of the P, I, D components had in your implementation and describe how the final hyperparameters were chosen.

In the PID controller, proportional portion will correct the error between the current measurement and target value since it will result in a control that will compensate this error. Integral portion will eliminate the steady state error since its value will accumulate as long as such error exists. The derivative portion will eliminate the overshoot since its value is proportional to the change of the cte value.

Here in this specific project, a PD controller is adopted to control the steering instead of a full PID version with Kp as 0.12 and Kd as 0.13. The reason to ignore due to the variating curvature of the track. Under this case as the vehicle is driven on the trac the integral error will always accumulate and will dominate over the P and D control.

The P and D hyper-parameters are determined from the following steps:

1. Starting with a P controller, tune the value Kp so that the vehicle can barely make the sharpest corner
2. Add Kd value (first guess will be half of Kp) and verify if vehicle is able to complete one lap
3. Tune the value of Kp and Kd (increase Kd if vehicle keep zigzaging on mild corner; decrease if vehicle gets unstable at certain corners or bridges) until the vehicle behavior is as expected

#### 2. Throttle control and other innovation points

Here in order to make d_error independent of vehicle speed, another PID controller is adopted to control the vehicle speed at 18 mph. Throttle controller has Kp as 0.5, Ki as 0.0001 and Kd as 0.01. This controller allows certain negative steady state error since it is not desire to have frequent braking and accelerating during steering. The steps of detailed tuning is similar to the steering controller.

Also here for steering, I choose a open loop controller (P only) between the start of vehicle and the moment when it reaches the target speed (18 mph). The reason is that during this period the vehicle speed is increasing and therefore the d_error term is strongly influenced by the current vehicle speed. In the [main.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-PID-Control-Project/src/main.cpp) a bool flag "is_stable" is introduced to indicate whether vehicle has reached the target speed. If it does, the steering controller will be re-initialized with the hyper-parameters mentioned above (line 63 in [main.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-PID-Control-Project/src/main.cpp)).

### Simulation

#### 1. The vehicle must successfully drive a lap around the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

It can be proved by running the simulator with the PID controller that vehicle is able to complete one lap with the current parameter tunings. 






