# Quadruped Manipulator NMPC for SAR and Doors
## Overview
The following branch includes the necessary files for simulating autonmous door opening with the AlienGo quadruped and the Z1 manipulator. In order to do this, the NMPC-WBC controller has been incorporated an end-effector force tracking constraint. In addition to this, two additional packages, `qm_planner` and `qm_vision` are included:
- `qm_planner`: This package includes two different planning files.

  - `GoToDoor.cpp`: Enables the robot to reach an initial position with respect to the door that can be fixed by the user from any given orientation and position from which the door is visible. By using visual references from the D435 camera base camera, the robot plans its reorientation trajectory.

  - `OpenDoor.cpp`: Enables the robot to detect the door handle and its orientation and track its 3D position estimation until grapping it. Then, the push door is opened following the planner trajectories.

- `qm_vision`: This package includes two YOLOv8 models: one for door detection, and the other one for handle classification and segmentation. The package includes four scripts:

  - `detect.py`: This script runs inference on the handle segmentation model, applies the calculated mask to the depth image from the arm camera and calculates the handle segmented point cloud.

  - `door_center.py`: This script runs inference on the door detection model, estimates the reference positions and yaw for the repositioning planner, and also generates some visualization features.

  - `door_params.py`: This script estimates the door swing radius and normal distance to the door wall plane.

  - `handle_centroid.py`: This script estimates the handle point cloud 3D centroid position and its orientation using RANSAC and PCA algorithms.

## Installation
After following the installation steps from the `main` branch, you will need to install a couple of extra dependencies for the computer vision package.

### Ultralytics YOLOv8
The computer vision package includes two YOLOv8 models for door detection and handle classification and segmentation. In order to use them, you will need to download and install the following [package](https://github.com/ultralytics/ultralytics):
```
pip install ultralytics
```
Note that in order to be able to install it you need at least Python 3.8 with PyTorch 1.8.

### Intel RealSense Gazebo Plugin
The quadruped has been equipped with two RGB-D cameras: one Intel RealSense D435 embedded in the front of the AlienGo, and one Intel RealSense SR305 mounted on the Z1 arm. If you want to use the vision package you will need to download  this specific Gazebo [plugin](https://github.com/pal-robotics/realsense_gazebo_plugin) from PAL Robotics. To use it clone the `melodic-devel` branch from the repository (valid in ROS Noetic) in the `src` folder of your workspace:
```
git clone https://github.com/pal-robotics/realsense_gazebo_plugin.git
```

### Build
After installing the previous dependencies into your workspace you can build the planner and vision packages with:
```
catkin build qm_planner qm_vision
```

## Bugs & Feature Requests
The project is still in the early stages of development and we welcome feedback. Please report bugs and request features using the [Issue Tracker](https://github.com/danisotelo/qm_door/issues).

## To Do
- [ ] Implement perception-NMPC for foot step optimization.
- [ ] Improve push door opening performance.
- [ ] Improve YOLO model for push/pull door classification.
- [ ] Support pull door and knob opening maneuvers.
