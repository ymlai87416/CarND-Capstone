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
    
The first column is x coordinate, and the second column is the y coordinate, while the
third column is the z coordinate (upward).
The final column represents the yaw angle when the car drives through this point.

## Waypoint loader

Waypoint loader reads the waypoint file and publishes a route in form of a list of waypoints which decelerate and stop at the last waypoint.

## Waypoint updater

Waypoint updater read the list of waypoint with linear velocity and decides the car should accelerate or decelerate depending on the
traffic condition.

The deceleration follows the square root function and is visualized in the following graph.

![alt text][image1]

For acceleration, it set the speed to the v<sub>target</sub> and the PID controller in twist controller executes
the acceleration.

## Waypoint follower

Waypoint follower consumes values published from waypoint updater and propose both linear velocity and angular velocity.

To calculate linear velocity: it uses the linear velocity of the next waypoint

To calculate angular velocity: 

* Find a linear equation which extrapolates the previous waypoint to the next waypoint
* Draw a circle around the current car position
* Find intersection points of a circle and the linear equation
* Set the target to the intersection point between the car and the next waypoint
* Find kappa (κ) = 2𝚫y / (𝚫s)<sup>2</sup>. κ has a unit of m<sup>-1</sup>
* Angular velocity = linear velocity * κ. Angular velocity has a unit of rad/s

For detail, please refer to `geometry_msgs::TwistStamped PurePursuit::go()` for details

## Twist controller

Twist controller takes the current velocity, proposed linear and angular velocity (from twist_cmd) to output
3 quantities: throttle, steering and brake.


Throttle is controlled by a PID controller, for implementation detail, please refer to `src/twist/twist_controller/pid.py`


Steering is controlled by a yaw controller and a PID controller, and the implementation is at `src/twist/twist_controller/yaw_controller.py`

The yaw controller performs the following
* Find out the angular_velocity<sub>current</sub> = angular_velocity<sub>propose</sub> / linear_velocity<sub>propose</sub> * 
linear_velocity<sub>current</sub>
* Check if any violation against max lateral acceleration
* Check if there is any violation against the limit of steering angle. (around +/- 8 in this project)
* Finally, it returns an angle in rad for steering.

Brake is controlled by the logic in `src/twist/twist_controller.py` line 85-91

* If the velocity is lower than 0.1ms<sup>-1</sup>, the car applies the brake at 700Nm too stop the car.
* If the throttle is lower than 0.1 and e(t) of PID control is small, consider the car is decelerating and apply
the brake accordingly.

### Throttle PID controller

Throttle PID controller controls the speed of the vehicle by throttle values.
It is very simple and does not need many parameters in order to work.

In this project, throttle PID controller has the following parameters
* Kp = 0.3
* Ki = 0.1
* Kd = 0

### Steering PID controller

Although `yaw_controller` calculates an angle base on sensor data and waypoints, sometimes it does not follow the track
perfectly. To come up with a more accurate steering angle, we have integrated a PID controller and it works
side by side with the `yaw_controller` like the following diagram. This PID controller consumes cross track
error and outputs a steering angle.

![alt text][image2]

The benefit of our design compares to yaw controller only solution is that our design is more tolerant to the change
of car parameters e.g. weight of the car.

Our design is also better than a PID controller only solution because the yaw controller provides an approximated
value which makes the controller more responsive. It is crucial when the car does not have much space to maneuver
and should be better at avoiding collisions.

