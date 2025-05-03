# Robot Arm

This repository contains an implementation of a robot arm for the "Introduction to Robotics" course at the University of Trento

## Getting started

The first step is to build everything using catkin
```bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

Then you should source the setup file
```bash
source ./devel/setup.bash
```


### Visualizing the robot using rviz
```bash
roslaunch robot_arm visualize.launch
```
