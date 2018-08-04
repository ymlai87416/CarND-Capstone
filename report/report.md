# Udacity Car Capstone project 

[//]: # (Image References)

[image1]: ../imgs/simulator.png "simulator"
[image2]: ../imgs/ros_code_structure.PNG "ros code structure"
[image3]: ../imgs/tl-detector-ros-graph.png "traffic light detector node"
[image4]: ../imgs/waypoint-updater-ros-graph.png "waypoint updater node"
[image5]: ../imgs/dbw-node-ros-graph.png "dbw node"

Presented by

Project leader:
* Muhammand Hassan - UTC+1 (Berlin)

Members:
* Lai Yiu Ming, Tom - UTC+8 (HK)
* Emre Sezginalp  - UTC+2 (Prague)
* Damain - UTC+1 
* Marvin - UTC+1 (Berlin)

## Project background
For this project, you'll be writing ROS nodes to implement core functionality of the autonomous vehicle system,
including traffic light detection, control, and waypoint following! You will test your code using a simulator, 
and when you are ready, your group can submit the project to be run on Carla.

The following is a system architecture diagram showing the ROS nodes and topics used in the project. 
You can refer to the diagram throughout the project as needed. The ROS nodes and topics shown in the diagram are 
described briefly in the Code Structure section below, and more detail is provided for each node in later 
classroom concepts of this lesson.

## Project setup


### Simulator

The simulator (ROS integration simulator) can be downloaded at [here](https://github.com/udacity/self-driving-car-sim/releases), 
there are 2 maps available. First one is `Highway`, and the second one is `Test Lot`.


![alt text][image1]

### Project setup


This project use Docker to develop and hence it is recommended to run this project inside a docker container.

1. [Install Docker](https://docs.docker.com/install/)

2. Build the docker container

    ```
    docker build . -t capstone
    ```

3. Run the docker file

    ```
    docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
    ```
    To enable X-display
    
    ```
    sudo xhost +local:docker
    sudo docker run -p 4567:4567 -v $PWD:/capstone -e DISPLAY=$DISPLAY -v /tmp/log:/root/.ros/ -v /tmp/.X11-unix:/tmp/.X11-unix --rm -it capstone
    ```

4. Run ROS command to kick start the program

    ```
    cd ros
    catkin_make
    source devel/setup.sh
    roslaunch launch/styx.launch
    ```

5. Run the simulator

### Project directories overview

In this project, there are multiple folders and serve difference purposes.

| Folder | Description |
| :------------ | :----------- |
| /data       | Contain waypoints files for 2 simulated scenario  |
| /img       | Contain image for read me and report  |
| /report       | Report for this project |
| /ros       | ROS catkin workspace  |
| /ros/devel       |  development folder |
| /ros/launch       | Contains launch files, 1 for simulator and 1 for site testing |
| /ros/src       | ROS nodes source files  |


## System architecture overview

### ROS

Please refer to [this link](./system_architecture.md) for further information.

### Individual component walk-thought

#### Speed and Steering

Please refer to [this link](./control.md) for further information.

#### Traffic light detection

Please refer to [this link](./preception.md) for further information.

## Result
