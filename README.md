### 1. installation and build the file

install dependent packages
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers ros-melodic-ros-control ros-melodic-ros-controllers

```

install gazebo_ros_control
```
sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
```
down load source file
```
git clone https://github.com/capstone-teama-ros/capstone1_ROS_gazebo
```
build the code
```
cd ~/capstone1_ROS_gazebo
catkin_make
```


### 2. running the code

map open

```
roslaunch map_generate import_world.launch
```

Spawn the robot

```
roslaunch p31 myrobot.launch
```

launch the controller_manager

```
roslaunch p31_control control.launch
```

detect and publish ball position & line detector info from camera image

```
rosrun ball_detection ball_detect_node
rosrun ball_detection line_detect_node
```

start the AI
```
rosrun data_integrate data_integration_node
```
