# Quadruped Manipulator NMPC for SAR and Doors
## Overview

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

## Bugs & Feature Requests
The project is still in the early stages of development and we welcome feedback. Please report bugs and request features using the [Issue Tracker](https://github.com/danisotelo/qm_door/issues).

## To Do
- [ ] Implement perception-NMPC for foot step optimization.
- [ ] Improve push door opening performance.
- [ ] Improve YOLO model for push/pull door classification.
- [ ] Support pull door and knob opening maneuvers.
