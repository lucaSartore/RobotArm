# Robot Arm

This repository contains an implementation of a robot arm for the "Introduction to Robotics" course at the University of Trento
## Video
[![Watch the demo video](https://img.youtube.com/vi/NH_AHPulZkg/0.jpg)](https://www.youtube.com/watch?v=NH_AHPulZkg)
## Project Description

An in depth report for this project can be seen [here](./readme.md)

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

### Running python scripts
To instead run one of the python scripts you can use
```bash
rosrun robot_arm main.py
```

## CLI description

The main.py has a small cli tool built into it that allows you to run the different part of the project
Here is a short documentation on the CLI:


### Usage

Run the default **full simulation**:

```bash
rosrun robot_arm main.py
```

Run a specific mode by passing it as an argument:

```bash
rosrun robot_arm main.py test_dynamics
rosrun robot_arm main.py test_dynamics_with_initial_velocity
rosrun robot_arm main.py test_inverse_kinematics
rosrun robot_arm main.py test_inverse_kinematics_with_postural
```

### Available Modes

* `run_full_simulation` – Full simulation pipeline (default)
* `test_dynamics` – Evaluate dynamics without initial velocity
* `test_dynamics_with_initial_velocity` – Dynamics test with initial motion
* `test_inverse_kinematics` – Basic inverse kinematics test
* `test_inverse_kinematics_with_postural` – Test IK with postural optimization


## Requirements

The project has a few extra requirements with respect to a standard ros project.
They are all listed in [requirements.txt](./src/robot_arm/requirements.txt).
In particular I installed a few libraries to parse and validate xml as well
as updating the numpy version to have support for the `typing` module.

