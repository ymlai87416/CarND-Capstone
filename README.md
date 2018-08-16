# Udacity Car Capstone project 

[//]: # (Image References)

[image1]: ./imgs/simulator.png "simulator"
[image2]: ./imgs/ros_code_structure.PNG "ros code structure"
[image3]: ./imgs/tl-detector-ros-graph.png "traffic light detector node"
[image4]: ./imgs/waypoint-updater-ros-graph.png "waypoint updater node"
[image5]: ./imgs/dbw-node-ros-graph.png "dbw node"
[image6]: ./imgs/result_highway_video.PNG "result highway"
[image7]: ./imgs/result_testlot_video.PNG "result testlot"
[image8]: ./imgs/result_rosbag_video.PNG "rosbag testlot"

Presented by

#### RoboCar team

Project leader:
<table style="border-collapse: collapse; border: none;">
<tr>
    <td>Lai Yiu Ming, Tom</td>
    <td>ymlai87416@gmail.com</td>
</tr>
</table>

Members:
<table style="border-collapse: collapse; border: none;">
<tr>
    <td>Muhammand Hassan</td>
    <td>hassan.nust@gmail.com</td>
</tr>
<tr>
    <td>Emre Sezginalp</td>
    <td>emre.sezginalp@gmail.com</td>
</tr>
<tr>
    <td>Damian Finnerty</td>
    <td>damfinn@gmail.com</td>
</tr>
<tr>
    <td>Marvin</td>
    <td>marv.ris@gmail.com</td>
</tr>
</table>

## Project background
For this project, our team writes ROS nodes to implement the core functionality of the autonomous vehicle system,
including traffic light detection, control, and waypoint following! We test our code using a simulator
and submit the project to Udacity to run it on Carla.

The following is a system architecture diagram showing the ROS nodes and topics used in the project.
The ROS nodes and topics shown in the diagram are
described briefly in the Code Structure section below, and more detail is provided for each node in later sections.

![alt text][image2]

## Project setup

### Project directories overview

In this project, there are multiple folders and serve different purposes.

| Folder | Description |
| :------------ | :----------- |
| /data       | Contain waypoints files for 2 simulated scenario  |
| /img       | Contain image for read me and report  |
| /model       | Traffic light detection for both simulated and real environments |
| /report       | Report for this project |
| /ros       | ROS catkin workspace  |
| /ros/devel       |  development folder |
| /ros/launch       | Contains launch files, one for simulator and one for site testing |
| /ros/src       | ROS nodes source files  |


### Project setup

This project uses Docker to develop and hence it is recommended to run this project inside a Docker container.

#### System requirement

##### Development environment
* Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir
* Processor: 3.0GHz processor or above
* 6GB System RAM
* Nvidia GPU card

##### Udacity Self-Driving Car Hardware Specs
* 31.4 GiB Memory
* Intel Core i7-6700K CPU @ 4 GHz x 8
* TITAN X Graphics
* 64-bit OS

#### Setup development environment

1. [Install Docker](https://docs.docker.com/install/)

2. [Install Nvidia-Docker](https://github.com/nvidia/nvidia-docker/wiki/Installation-(version-2.0))

3. Build the docker container

    ```
    docker build . -t capstone
    ```

4. Run the docker file

    ```
    docker run --runtime=nvidia -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
    ```

5. To enable X-display in docker environment
    
    ```
    sudo xhost +local:docker
    sudo docker run --runtime=nvidia -p 4567:4567 -v $PWD:/capstone -e DISPLAY=$DISPLAY -v /tmp/log:/root/.ros/ -v /tmp/.X11-unix:/tmp/.X11-unix --rm -it capstone
    ```

6. Run ROS command to kick start the program

    ```
    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch
    ```

7. Run the simulator

### Simulator

The simulator (ROS integration simulator) can be downloaded at [here](https://github.com/udacity/self-driving-car-sim/releases),
there are 2 maps available. `Highway` and `Test Lot`.

Self-driving car is a complex system and consist of the following important modules in order to drive properly.

![alt text][image1]

In this project, we are going to create a self-driving car which is able to follow waypoints, and stop
 in front of a red light.

This project is implemented in ROS (Robot operating system). Please refer
to [this link](./report/system_architecture.md) for details.

### Individual component walk-thought

#### Speed and Steering

Please refer to [this link](./report/control.md) for further information.

#### Traffic light detection

Please refer to [this link](./report/perception.md) for further information.

## Result

This project composes of 3 parts: Running on the simulator, pre Carla testing on ROS bag and
testing on Carla.

#### Running on the simulator

[<img src="https://github.com/ymlai87416/CarND-Capstone/blob/master/imgs/result_highway_video.PNG">](https://youtu.be/K3YOUEuKAwQ)

[<img src="https://github.com/ymlai87416/CarND-Capstone/blob/master/imgs/result_testlot_video.PNG">](https://youtu.be/J22OQShw-7o)

#### Pre Carla testing on ROS bag

[<img src="https://github.com/ymlai87416/CarND-Capstone/blob/master/imgs/result_rosbag_video.PNG">](https://youtu.be/lCfDJDUgrS8)

#### Testing on Carla

TBA