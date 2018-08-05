# Control

[//]: # (Image References)

[image1]: ../imgs/decelerate_graph.png "declerate"
[image2]: ../imgs/steering_control.png "steering control"

## Waypoint file structure

There are 2 waypoint files to guide the car to drive around those 2 simulations `Highway` and `Test Lot`. They are:
* /data/wp_yaw_const.csv
* /data/churchlot_with_cars.csv

Here is the file structure 

    912.781,1128.67,0.0,0
    913.131,1128.67,0.0,0.233704
    913.375,1128.68,0.0,0.452972
    913.885,1128.68,0.0,0.426353
    914.288,1128.68,0.0,0.433952
    ......
    
The first column represents x coordinate, and the second column represent the y coordinate, while the
third column represent the z coordinate (upward).
The final column represent the yaw angle when the car drives through this point.

## Waypoint loader

Waypoint loader reads the waypoint file and return a route which stop at the last waypoint.

## Waypoint updater

Waypoint updater read the list of waypoint with linear velocity and decides the car should accelerate or decelerate depending on the
traffic condition.

The deceleration follows the square root function and the below graph visualize it.

![alt text][image1]

The acceleration function is controlled by PID controller of the twist controller and is discuss later.  

## Waypoint follower

Waypoint follower consumes the result of waypoint updater and propose both linear velocity and angular velocity.

To calculate linear velocity: it get the linear velocity of the next waypoint

To calculate angular velocity: 

* Find a linear equation which extrapolate the previous waypoint to the next waypoint
* Draw a circle around the current car position
* Find intersection points of a circle and the linear equation
* Set the target to the intersection point between the car and the next waypoint
* Find kappa (κ) = 2𝚫y / (𝚫s)<sup>2</sup>, κ has a unit of m<sup>-1</sup>
* Angular velocity = linear velocity * κ and has a unit of rad/s<sup>2</sup>

For detail, please refer to `geometry_msgs::TwistStamped PurePursuit::go()` for details

## Twist controller

Twist controller takes the the current velocity, proposed linear and angular velocity (from twist_cmd) to output
3 quantities: throttle, steering and brake.

Throttle is controlled by a PID controller, for implementation detail, please refer to `src/twist/twist_controller/pid.py`

Steering is controlled by a yaw controller and a PID controller, and the implementation is at `src/twist/twist_controller/yaw_controller.py`

The yaw controller performs the following
* Find out the angular_velocity<sub>current</sub> = angular_velocity<sub>propose</sub> / linear_velocity<sub>propose</sub> * 
linear_velocity<sub>current</sub>
* Check if the any violation against max lateral acceleration
* Check if there are any violation against the limit of steering angle. (around +/- 8 in this project)
* Finally, it return an angle in rad for steering.

Brake is controlled by a the logic in `src/twist/twist_controller.py` line 65-71

* If the velocity is lower than 0.1ms<sup>-1</sup>, the car apply the brake at 700Nm too stop the car.
* If the throttle is lower than 0.1 and e(t) of PID control is small, consider the car is decelerating and apply
the brake accordingly.

* Warn: break_deadband is not used.

### Tuning throttle PID controller


### Tuning steering PID controller

Although `yaw_controller` calculate an angle base on sensor data and waypoints, sometimes it does not follow the track
perfectly. To  

![alt text][image2]

