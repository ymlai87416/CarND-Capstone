


# System architecture

[//]: # (Image References)

[image1]: ../imgs/self_driving_stack.PNG "driving stack"
[image2]: ../imgs/ros_code_structure.PNG "ros code structure"
[image3]: ../imgs/tl-detector-ros-graph.png "traffic light detector node"
[image4]: ../imgs/waypoint-updater-ros-graph.png "waypoint updater node"
[image5]: ../imgs/dbw-node-ros-graph.png "dbw node"

This project is implemented in ROS (Robot operating system), and below graph should ROS nodes which make up of
our project. In this section, we present each node in detail.

![alt text][image2]

## Nodes

### Perception - Obstacle Detection

This node is not implemented in this project.

### Perception - Traffic light detection node (tl_detector)

This package contains the traffic light detection node: `tl_detector.py`. This node takes in data from the 
`/image_color`, `/current_pose`, and `/base_waypoints` topics and publishes the locations to stop for red traffic 
lights to the `/traffic_waypoint` topic.


The `/current_pose` topic provides the vehicle's current position, and `/base_waypoints` provides a complete list of 
waypoints the car will be following.


This ROS node has a traffic light detection node and a traffic light classification node. Traffic light detection
takes place within tl_detector.py, whereas traffic light classification takes place within
`../tl_detector/light_classification_model/tl_classfier.py`.

![alt text][image3]

### Planning - Waypoint Loader (waypoint_loader)

A package which loads the static waypoint data and publishes to `/base_waypoints`.

Please refer to [Control section](./control.md) for waypoint file format.

### Planning - Waypoint updater node (waypoint_updater)

This package contains the waypoint updater node: `waypoint_updater.py`. The purpose of this node is to update the
target velocity property of each waypoint based on traffic light and obstacle detection data. This node
subscribes to the `/base_waypoints`, `/current_pose`, `/obstacle_waypoint`, and `/traffic_waypoint` topics, and
publish a list of waypoints ahead of the car with target velocities to the `/final_waypoints` topic.

![alt text][image4]

### Control - DBW node (twist_controller)

Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic 
control. This package contains the files that are responsible for control of the vehicle: the node `dbw_node.py` 
and the file `twist_controller.py`, along with a pid and lowpass filter that you can use in your implementation. 
The dbw_node subscribes to the `/current_velocity` topic along with the `/twist_cmd` topic to receive target 
linear and angular velocities. Additionally, this node will subscribe to `/vehicle/dbw_enabled`, which indicates 
if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the 
`/vehicle/throttle_cmd`, `/vehicle/brake_cmd`, and `/vehicle/steering_cmd` topics.

![alt text][image5]


### Control - Waypoint follower (waypoint_follower)

A package containing code from Autoware which subscribes to `/final_waypoints` and publishes target vehicle linear 
and angular velocities in the form of twist commands to the `/twist_cmd topic`.

## Topic

Here is a list of messages used by the system


| Topic | Message type | Description |
| :------------ | :----------- | :----------- |
| /base_waypoints       | styx_msgs/Lane            | From waypoint loader, represent a track by a list of waypoints         |
| /current_pose         | geometry_msgs/PoseStamped | From car/simulator, represent the car current position and orientation          |
| /current_velocity     | geometry_msgs/TwistStamped| From car/simulator, represent the car current velocity          |
| /final_waypoints      | styx_msgs/Lane            | From waypoint updater node, consumed by waypoint follower to become steering angle, throttle and brake         |
| /image_color OR /image_raw         | sensor_msgs/Image         | From car/simulator, for perception module to identify traffic light and obstacle          |
| /obstacle_waypoints   | ???                       | From perception module, represent obstacle by coordinates          |
| /traffic_waypoint     | std_msgs/Int32            | From perception module, represent traffic light by traffic light stop-lines and light signal          |
| /twist_cmd            | geometry_msgs/TwistStamped| From waypoint follower node, consumed by DBW node to become steering angle, throttle and brake  |
| /vehicle/brake_cmd    | dbw_mkz_msgs/BrakeCmd     | From DBW node, control the car break   |
| /vehicle/dbw_enabled  | std_msgs/Bool             | From car/simulator, tell that the car is under human or system control          |
| /vehicle/steering_cmd | dbw_mkz_msgs/SteeringCmd  | From DBW node, control the car steering angle          |
| /vehicle/throttle_cmd | dbw_mkz_msgs/ThrottleCmd  | From DBW node, control the car throttle          |


## Messages

These are messages used in this project.

#### dbw_mkz_msgs/BrakeCmd

```
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
uint8 CMD_TORQUE=3
float32 TORQUE_BOO=520
float32 TORQUE_MAX=3412
float32 pedal_cmd
uint8 pedal_cmd_type
bool boo_cmd
bool enable
bool clear
bool ignore
uint8 count
```

#### dbw_mkz_msgs/SteeringCmd

```
float32 steering_wheel_angle_cmd
float32 steering_wheel_angle_velocity
bool enable
bool clear
bool ignore
bool quiet
uint8 count
```

#### dbw_mkz_msgs/ThrottleCmd

```
uint8 CMD_NONE=0
uint8 CMD_PEDAL=1
uint8 CMD_PERCENT=2
float32 pedal_cmd
uint8 pedal_cmd_type
bool enable
bool clear
bool ignore
uint8 count
```

#### geometry_msgs/PoseStamped
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
```

#### geometry_msgs/TwistStamped

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Twist twist
  geometry_msgs/Vector3 linear
    float64 x
    float64 y
    float64 z
  geometry_msgs/Vector3 angular
    float64 x
    float64 y
    float64 z
```

#### sensor_msgs/Image

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
```

#### styx_msgs/Lane

```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
styx_msgs/Waypoint[] waypoints
  geometry_msgs/PoseStamped pose
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Pose pose
      geometry_msgs/Point position
        float64 x
        float64 y
        float64 z
      geometry_msgs/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
  geometry_msgs/TwistStamped twist
    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/Twist twist
      geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z
```
