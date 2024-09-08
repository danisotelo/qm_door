# Quadruped Manipulator NMPC for SAR and Doors

## Overview

This project develops a <strong>NMPC-WBC</strong> (Nonlinear Model Predictive Control and Whole-Body Control) framework for a quadruped manipulator. Based on [OCS2](https://github.com/leggedrobotics/ocs2) and [ros-control](http://wiki.ros.org/ros_control), it facilitates tasks such as whole-body planning, end-effector motion tracking, and maintaining stability under force disturbances. The project builds upon the remarkable open-source contributions by [SkyWoodz](https://github.com/skywoodsz/qm_control) and [QiayuanL](https://github.com/qiayuanl/legged_control). The present project adapted this previous work for the <strong>Unitree AlienGo</strong> quadruped together with the <strong>Unitree Z1</strong> arm manipulator, and is built in <strong>ROS Noetic</strong>.

The controller was adapted and tested for different Search and Rescue (SAR) environments, including [door opening](https://github.com/danisotelo/qm_door/tree/force-tracking), though the project is <strong>still in development</strong>. Additional vision and planner modules were developed for the door opening task. The repository has <strong>three branches</strong> with different objectives:

- **`main`**: This branch contains all necessary files for running simulations on the quadruped manipulator in **SAR scenarios**. It enables control of the quadruped manipulator both as a combined system and as separate systems (independent controllers for the AlienGo and the Z1 arm).
- **`force-tracking`**: This branch is similar to the main but incorporates an end-effector's force tracking constraint into the NMPC optimization problem and as a task in the WBC. This branch includes the **door opening simulation**. It also includes the vision and planner modules, explained in more detail in the branch documentation.
- **`hw`**: This branch is dedicated to deployment on the actual system, including specific files employing the [Z1 SDK](https://github.com/unitreerobotics/z1_sdk). Some UDP communication issues were found.



## Installation

### Source code

The source code is hosted on GitHub: [danisotelo/qm_door](https://github.com/danisotelo/qm_door/issues).

```
git clone https://github.com/danisotelo/qm_door.git
```

### OCS2
OCS2 is a huge monorepo, it is not necessary to compile it completely. Only `ocs2_legged_robot_ros` and its dependencies are required, as described below.

1. You are supposed to clone OCS2, [pinocchio](https://github.com/stack-of-tasks/pinocchio), and [hpp-fcl](https://github.com/humanoid-path-planner/hpp-fcl) repositories, as described in the documentation of OCS2.
  ```
  # Clone OCS2
  git clone https://github.com/leggedrobotics/ocs2.git
  # Clone pinocchio
  git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
  # Clone hpp-fcl
  git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
  # Clone ocs2_robotic_assets
  git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git
  # Install dependencies
  sudo apt install liburdfodom-dev liboctomap-dev libassimp-dev
  ```
2. Compile the `ocs2_legged_robot_ros` package with [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/) instead of `catkin_make`. It will take you about ten minutes.
  ```
  catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
  catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization
  ```


### Build

## Quick Start

## Framework

## Bugs & Feature Requests

The project is still in the early stages of development and we welcome feedback. Please report bugs and request features using the [Issue Tracker](https://github.com/danisotelo/qm_door/issues).

## To Do

- [ ] Add gripper controller
- [ ] On-run step height and target velocity update
- [ ] Integrate navigation pipeline

## References

This project is built on top of the following repositories:

- **QM Control by Skywoodsz**: A quantum mechanics control library that influenced our approach to problem-solving. Check it out [here](https://github.com/skywoodsz/qm_control).

- **Legged Control by QiayuanL**: This repository provides advanced algorithms for legged robot control which were instrumental for our developments. Visit their project [here](https://github.com/qiayuanl/legged_control).
