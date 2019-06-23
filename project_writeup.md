# CarND-Path-Planning-Project

   
## 1. Introduction
This is the report for the project Path Planning of the Udacity Self Driving Car Nanodegree.

The project was quite challenging (if not the *most* challenging), but in the end my code managed to meet the goal of the rubric (driving at least 6946m without any forbidden event).

The challenges encountered dealt mainly of inaccuracy on transformation from Frenet coordinates to cartesian coordinates, this will be explained in detail below.


[//]: # (Image References)

[image1]: ./figures/Figure_1_Pipeline_small.png "Project Pipeline"
[image2]: ./figures/02_Simulator_small.png  "Simulator"
[image3]: ./figures/03_Frenet_small.png  "Frenet"
[image4]: ./figures/04_Constant_Trajectory_small.png "Constant Trajectory"
[image5]: ./figures/05_Frenet_error_small.png "Frenet conversion error"
[image6]: ./figures/06_q_poly_small.png "Quintic Polynomial function"
[image7]: ./figures/07_Matrix_small.png "Quintic Polynomial Matrix"
[image8]: ./figures/08_calculations_small.png "Spline calculations"

05_Frenet_error_small.png

## 2. Simulator
The project is based on controlling a vehicle in the Udacity Term 3 Simulator.

The green line showed is the trajectory, with the green spheres representing the points are path planner provides to the simulator. Each point will be visited by the car every 20ms.
![alt text][image2]


## 3. Pipeline
The basic pipeline is the one as described in the lessons 9 "Behavior Planning".
![alt text][image1]

The blocks enclosed in a green frame are the ones to be developed. The other blocks are already given by the simulator.

### 3.1 Provided modules
#### 3.1.1 Motion Control
The motion control is already programmed in the simulator, an it is an **ideal controller**. This means we have to pay attention to not exceeding the maximum acceleration (and jerk) given by the rubric, which in our case was 10m/s² and 10m/s³.

#### 3.1.2 Sensor fusion
The sensor fusion layer is also transferred from the simulator and deals with a list of objects (other cars) which are travelling in the ego or neighbouring lanes. These objects represent a coordinate of the center of the car. Additional information (such as velocity) is given, which it is necessary for trajectory generation. Additionally, the velocity is given with its x and y components (in map coordinates), in case we want to detect a car changing lanes.

#### 3.1.3 Localization.
The localization layer is also provided by the simulator. In each frame we know the x and y position of our car, as well as the speed and heading (in degrees). There are also some other other useful magnitudes as s and d in Frenet Coordinates.

Note that the heading of the vehicle did not seem to be very consistent. More on this will be explained in the trajectory generation.

The map data is also provided as a list of sparse waypoints located in the center of the highway (the yellow line), which are provided as a .csv file. The map information is necessary to calculate the trajectory and correct from Frenet coordinates to map coordinates.

## 4. Developed modules

### 4.1 Trajectory generation

The trajectory generation was the most difficult module to implement, this was also noted in several forums from other students.

I started by watching the Project Q&A by David Silver and Aaron Brown. There it was presented an approach of using splines to generate a smooth trajectory. The problem I saw with this approach (also briefly noted by the Q&A) was that the distance between points is approximated. I decided to explore first the method (Quintic Polynomial Solver) since it seemed more exact.

Finally I gave up this approach, since the provided function for converting Frenet coordinates to map coordinates showed errors which will be described below. Some other students approached the problem by generating a map with more waypoints.

So finally I sticked with the approach described by Aaron, which will be explained below.

#### 4.1.1 A word on Frenet coordinates
To calculate the trajectory, we are using Frenet coordinates. These are very useful, since we deal with the longitudinal position of the car and the lateral position of the car. The curves of the highway are then abstracted and we can figure it out as a straight highway. 

![alt text][image3]

Using Frenet, a straight trajectory is simply an increasing value of s and constant value of d. 

The s value increment is calculated using the velocity constraint (assuming the start and end state of car are constant). 

For example, to drive at the required 50 mph (22.352 m/s) speed and assuming each point shall be visited in 0.02ms, the distance between points shall not exceed:

ds = 22.352 m/s * 0.02 s = 0.447 m

The waypoints are located in the yellow line, so the distance to this line provides the lane mapping. Since we know the lane width to be 4 meters, driving on the center lane will require a constant d value of 6m.

![alt text][image4]

Since the controller understands map coordinates, we need to transform back to provide useful data for the simulator.

#### 4.1.2 Problems encountered with the Frenet transformations

The helper module provided a function to convert from Frenet coordinates to map coordinates. This function showed however errors in curves.

As seen in the figure, the transformation works if the car is driving along the same path as the waypoints. If the car is displaced laterally, the car will drive longer s values in case of left turns or shorter s values in case of right turns.

![alt text][image5]

This ended up producing jumps in case of left curves or even jumps back in case of right curves, so an exact calculation of the deltas for s was not possible.

I tried different approaches to solve the issue, but in the end due to time constraints I gave up. If more time would be available, it would be a good challenge to provide an exact function to transform the vehicle Frenet Coordinates to Map coordinates.


#### 4.1.3 First approach: Quintic Polynomial solver
The first idea was to follow the approach of the lesson and generate a Quintic Polynomial solver. This approach consists of following steps, applied for both s and d:

1. Determine the start state of the vehicle (position, speed and acceleration). This is known from the motion controller.
2. Determine the end state of the vehicle (position, speed and acceleration). This is the desired end state. For example, for a lane change, the initial state would be d = 6m (from center lane) to d=2m (to leftmost lane).
3. The position variable can be represented as a function over time. The theory states that a jerk free path can be represented as a fifth polynomial.
![alt text][image6]
4. The problem, then, is to find the coefficients of this polynomial to satisfy the initial and end position. Assuming that the initial time is 0, the coefficients can be found using the formula:
![alt text][image7]
5. Convert the (s,d) coordinates to map coordinates.

As stated, the problem with this approach was the transformation from Frenet to Map coordinates, which then degraded the s calculation. Therefore this approach was discarded, until a proper fix for the transformation can be found.

#### 4.1.4 Final approach: using splines

The final approach for the trajectory was implemented according to the Q&A concept.

This concept consists on following steps:

1. Create five "anchor points" to calculate trajectory. The anchor points are: previous position of car (0.02ms before), current position of car, and three more points with the desired lateral position at fixed s distances. (In my case I experimented with 50.0, 80.0 and 110.0 to achieve a smooth trajectory, not producing violations due to sudden lane change).

2. Convert these five anchor points to map coordinates using the Frenet Transformation. Since we will calculate the increments on position later, the "jumps" due to the error in the function can be ignored.

3. Convert these five points in map coordinates to car coordinates. This is essential because the spline library can only deal with increasing x-values. Otherwise we would have areas in the map where the spline library would produce an error.

4. Find a matching spline for these five points.

5. Calculate the increment needed for this trajectory. Here is where an approximation is being used, which makes this solution not very elegant. To calculate the desired increment, we assume that the y component of the curve is so small that we can approximate the s increment using the x-increment. The increment in position is then given as explained in (4.1.1).

Steps 3, 4 and 5 are shown in the following graphic:
![alt text][image8]


6. Using these increments, and the spline library, we find the y coordinates for each x-increment. Note that we start the interpolation from the last points of the last trajectory, in order to avoid discontinuities.

7. Convert the x- and y-points to map coordinates.

Note that in order to convert to map coordinates we use the heading angle of the vehicle, **not as given by the simulator**, but as calculated from the initial two points of the trajectory. I need to mention that this mismatch was also not clear when dealing with the project and was inferred from the Q&A video.

This approach is working good assuming the spline has a smooth trajectory in the lateral direction of the vehicle.

To sum up, the trajectory will consist in:
1. Whichever number of points left by the previous calculation, as acquired from the simulator.
2. Remaining points to achieve the requested speed.

#### 4.1.5 Solving acceleration and deceleration

Note that the spline method works fine when dealing with a constant state of velocity, but will fail if the vehicle is accelerating or decelerating. 

In order to solve this, we define a *goal velocity*, but the *actual velocity* is allowed to change at a fixed rate. This rate is found out to be 0.25 for deceleration and 0.14 for acceleration. The reason of having two different constraints is explained as follows:

1. The current planner does not perform a lane change with constant speed. This means it can accelerate while changing lanes, which can introduce an additional error.

2. The vehicle will decelerate only when encountering an obstacle in front of it. A stronger deceleration was needed in order to avoid colliding with sudden braking vehicles or vehicles which change their lanes too fast.

### 4.2 Prediction
For the prediction phase, I made following assumptions:
1. Vehicles will not change lanes.
2. Vehicle travel at constant speed (acceleration is not provided by the fusion).
3. I use a conservative approach to minimize collisions.

The calculation of trajectories was therefore straightforward. By using the list of the fusion I get for each object a position and a velocity. 

I then interpolate each position using the speed for each point of my trajectory. 

For left or right lanes, there must not be any vehicle 10m behind and 40m in front. In case of violation to this, I set a flag to indicate that either left or right the lane is busy.

For ego lane,  I just detect if there is vehicle ahead. In that case, I store its speed which will be used later to set a new goal speed to avoid a collision.

### 4.3 Behaviour planner

The behaviour planner was implemented using a finite state machine (FSM).

I created four states:
START_STATE: Initial state, here I change directly to the DRIVE_AHEAD state.

DRIVE_AHEAD: The car drives forward at the desired maximum allowable speed.

DRIVE_FOLLOW: A vehicle is in front of our car. Our vehicle will set the goal speed to match the one of the vehicle ahead and will accept in this state conditions for a lane change.

CHANGE_LEFT: The desired lane is set to the left adjacent lane. Then we switch to DRIVE_AHEAD state.

CHANGE_RIGHT: The desired lane is set to the right adjacent lane. Then we switch to DRIVE_AHEAD state.


In this case I am not using cost functions to change from a state to another. I will verify that conditions are safe. Note that a left lane change is preferred over a right lane change if both conditions are met.


## 5. Conclusions and reflection
As stated before, this project was the most challenging of the whole Nanodegree. In my opinion because there is a signficant effort in programming low level functions. The results are rewarding, since we can see in the simulator how our car is actually *driving*!

The path planning problem is a very difficult task and here we dealt only with a simple path planner for a sparse environment as a highway. This project raised the awareness on this.

### 5.1 Drawbacks and limitations of my solution
I need to say that my limiting factor was time. So therefore I would like to include a list of known limitations of my solution:
1. The prediction stage should be improved by detecting vehicles changing lanes. For this the velocity components can be used.
2. The prediciton stage assigns objects to either one lane or the other. Objects occupying two lanes (for example when slowly overtaking) cannot be detected and might cause a collision.
3. The behaviour stage should be improved with cost functions to optimize the driving strategy.
4. The path planner still have some glitches in the performance, some manouvers (very sporadically) produce a violation of acceleration. Therefore I have to reduce minimally the speed.

I was nevertheless able to achieve the goal of driving at least one round without any incident.

### 5.2 Proposed improvements for this project
As an improvement suggestion, it would be great to have a somehow more precise Frenet to Cartesian function to have more freedom in the solution and save some time of implementation.