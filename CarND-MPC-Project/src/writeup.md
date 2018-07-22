# **Model Predictive Control (MPC) Project**

## Writeup

---

####Build a PID Controller Project

The goals / steps of this project are the following:
- Implement MPC controller in C++
- Tune the hyper-parameters of the controller to drive a virtual vehicle around the designated track

## Rubric Points
#### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/896/view) individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one

You're reading it! and here is a link to my [project code](https://github.com/Angelenos/CarND-Term2/tree/master/CarND-MPC-Project/src)

### Implementation of Controller

#### 1. The Model

Implementation can be found in [MPC.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-MPC-Project/src/MPC.cpp).

##### i. State

The input state has 6 items:

|	Index	|	Description					| 
|:-----------:	|:--------------------------------------------:	| 
|	0		|	Vehicle position in x-axis (local)	|
|	1		|	Vehicle position in y-axis (local)	| 
|	2		|	Vehicle orientation (local)		| 
|	3		|	Vehicle speed					| 
|	4		|	CTE (local)					| 
|	5		|	Error in vehicle orientation (local)	| 

Here all all state variables except vehicle speed are transferred from the global coordinates with respect to the map into the local coordinate with respect to the vehicle. In the local coordinate x-axis is along the direction that vehicle drivers and y-axis is perpendicular to it in the plane that vehicle drives. Coordinate conversion is performed with the function coord_trans in line 41 of [main.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-MPC-Project/src/main.cpp) before sent to the solver. Details will be covered later in the MPC preprocessing section.

##### ii. Actuator

2 Actuators are accepted by the solver: steering angle and accelerations. Here both actuators are normalized into the range of [-1, 1], while in reality steering angle has the range of [-25, 25] in degrees and accelerations of [-15, 15] in mph. Limit of steering angle is determined by the physical model in the simulator. Limit of acceleration/brake is estimated by performing 0-60 mph WOT acceleration and 60-0 mph full brake.

Limits of actuators serve as solver constraints and are specified in the line 176 to 186 from [MPC.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-MPC-Project/src/MPC.cpp).

##### iii. Update Equations

Update equations are modified from the phyiscal vehicle models mentioned in [Lesson 18](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/af4fcd4f-eb1f-43d8-82b3-17bb1e71695f/concepts/a62153eb-458b-4c68-96fc-eeaa3b4aeaa6).

\begin{equation}
x_{t+1} = x_t + v_t \cos{\psi_t}*dt
\tag{1}
\end{equation}

\begin{equation}
y_{t+1} = y_t + v_t \sin{\psi_t}*dt
\tag{2}
\end{equation}

\begin{equation}
\psi_{t+1} = \psi_t -\frac{v_t}{L_f}\delta_t*dt
\tag{3}
\end{equation}

\begin{equation}
v_{t+1} = v_t + a_t*dt
\tag{4}
\end{equation}

\begin{equation}
\mathbb{cte}_{t+1} = \mathbb{cte}_t + v_t\sin{(e\psi_t)}*dt
\tag{5}
\end{equation}

\begin{equation}
e\psi_{t+1} = e\psi_t - \frac{v_t}{L_f}\delta_t*dt
\tag{6}
\end{equation}

Here note that since the simulator defines the steering angle in an opposite way compared with the Udacity lecture, the original + is replaced with - in equation 3 and 6 to keep consistency.

#### 2. Timestep Length and Elapsed Duration (N & dt)

Timestep of 0.05s and total number of prediction points of 15 are chosen in this project.

Timestep of value 0.05s guarantees the solver resolution is smaller than the latency and prevents sampling bias or controller unable to capture the change within the unit timestep of latency.

Elapsed duration of 15 total prediction points balances sufficient accuracy and solver speed. 

#### 3. Polynomial Fitting and MPC Preprocessing

##### i. MPC Preprocessing

Since CTE is defined in the vehicle local coordiatee, it is necessary to convert all vehicle measurements into the vehicle local coordinates. Here I implement the funciton coord_trans in line 41 of [main.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-MPC-Project/src/main.cpp). It received the current vehicle positions in global coordinate and the points that needs to be converted.

$$\begin{bmatrix}
x_\mathbb{loc} \\
y_\mathbb{loc} \\
\end{bmatrix} = \begin{bmatrix}
\sin\psi & \cos\psi \\
-\cos\psi & \sin\psi \\
\end{bmatrix}\begin{bmatrix}
x - x_\mathbb{veh} \\
y - y_\mathbb{veh} \\
\end{bmatrix}$$

Here **the vehicle orientation is the unity one with 0 degree at the true north**. Reference points ptsx and ptsy are converted right after received from simulator before the polynomial fitting. All state variables sent to the solver are in vehicle local coordinate as well to keep consistency.

##### ii. Polonomial Fitting

A polynomial fitting with the order of 3 are adopted to interpolate the reference points. Implementation can be found in line 51 of [main.cpp](https://github.com/Angelenos/CarND-Term2/blob/master/CarND-MPC-Project/src/main.cpp).

#### 4. Model Predictive Control with Latency

After verification, it can be found that the current MPC controller is able to fix the error caused by 100 ms of latency without any further optimization. Hence no special actions are take to deal with the latency.

### Simulation

#### 1. The vehicle must successfully drive a lap around the track.

It can be verified by running the model in simulator that this controller is able to complete one lap without running out of track with target speed of 100 mph. During corner the controller may slow down the vehicle so it can pass the corner safely but during the straight line the controller is also able to keep the vehicle as close to the target speed as possible.





