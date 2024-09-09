# Quadruped Manipulator NMPC for SAR and Doors

<p align="center">
  <img src="https://github.com/danisotelo/qm_door/blob/main/docs/tunnel.gif" height="280px" />
  <img src="https://github.com/danisotelo/qm_door/blob/main/docs/door.gif" height="280px" />
</p>

## Overview

This project develops a <strong>NMPC-WBC</strong> (Nonlinear Model Predictive Control and Whole-Body Control) framework for a quadruped manipulator. Based on [OCS2](https://github.com/leggedrobotics/ocs2) and [ros-control](http://wiki.ros.org/ros_control), it facilitates tasks such as whole-body planning, end-effector motion tracking, and maintaining stability under force disturbances. The project builds upon the remarkable open-source contributions by [SkyWoodz](https://github.com/skywoodsz/qm_control) and [QiayuanL](https://github.com/qiayuanl/legged_control). The present project adapted this previous work for the <strong>Unitree AlienGo</strong> quadruped together with the <strong>Unitree Z1</strong> arm manipulator, and is built in <strong>ROS Noetic</strong>.

The controller was adapted and tested for different Search and Rescue (SAR) environments, including [door opening](https://github.com/danisotelo/qm_door/tree/force-tracking), though the project is <strong>still in development</strong>. Additional vision and planner modules were developed for the door opening task. The repository has <strong>three branches</strong> with different objectives:

- **`main`**: This branch contains all necessary files for running simulations on the quadruped manipulator in **SAR scenarios** (obstacles, pallets, ramps, stairs, tunnels, V-chimney, doors, etc.). It enables control of the quadruped manipulator both as a combined system and as separate systems (independent controllers for the AlienGo and the Z1 arm).
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

Build the source code of `qm_door` by:

```
catkin build qm_door
```

## Quick Start

1. In one terminal run the Gazebo simulation you want. Several different worlds are available in the `qm_gazebo`package, mainly categorized into two folders corresponding to **combined** or **separated** systems (same world name but finished in `_mpc`). This election depends on whether it is desired to control the whole quadruped manipulator as one unique robot and thus one NMPC-WBC controller; or it is desired the MPC controller to only include the quadruped and control the robotic arm with separate PID controllers. Note that **gripper control is currently only supported for separated systems**. For example:

    - Combined system:

      ```
      roslaunch qm_gazebo stairs_world.launch
      ```

    - Separated system:

      ```
      roslaunch qm_gazebo stairs_world_mpc.launch
      ```

2. Then, in a second terminal load the NMPC controller. The controller to load will depend on the selected control strategy. For controlling the combined system run:
    ```      
    roslaunch qm_controllers load_controller.launch
    ```
    In case it is desired to control the robots as separated systems run:
    ```      
    roslaunch qm_controllers load_controller_mpc.launch
    ```

    Next, you should click on the **play button** in the opened Gazebo window to initialize the simulation. Once the controller has been loaded, from the same terminal you can open the `rqt_controller_manager` to load and activate the loaded controller:
    ```      
    rosrun rqt_controller_manager rqt_controller_manager
    ```

3. The launch of the gazebo simulation will have opened the corresponding RViz visualization. The robot can be controlled with velocity commands, but if you want to control it with the end-effector position from RViz you should run in a third terminal:
    ```      
    roslaunch qm_controllers load_qm_target.launch
    ```
4. While controlling the robot with the end-effector position, if you want to change the gait pattern (eleven different schedules are available, "trot" is used in the example), simply run in a separate terminal:
    ```      
    rostopic pub /gait_command_topic std_msgs/String "trot"
    ```

You can program in a higher-level the trajectories you want the robot to follow in the `qm_planner` package. An example for a simple circular trajectory is included. The constraints imposed to both the robot base and the arm while following these end-effector trajectories, can be modified from `QmTargetTrajectoriesPublisher_node.cpp`. 

In case you want to spawn the robot's URDF in a different position or orientation in the Gazebo simulation you can do this by modifying the parameters in `StartingPosition.h` from the `qm_controllers` package. Note that these parameters should coincide with the ones of the Gazebo launch file of the `qm_gazebo` package for the controllers to work properly.

## Framework

The system framework diagram is shown below:
![](docs/framework.png)

1. The vision module determines the references that serve as input to the planner module. Note that these two modules are only implemented in the `force-tracking` branch for the **door opening task**.

2. The robot torso's desired velocity or position goal is converted to state trajectory and then sent to the NMPC. These desired trajectories can use as input both the user (from RViz) and the planner module.

3. The NMPC will evaluate an optimized system state and input.

4. The Whole-Body Controller (WBC) figures out the joint torques according to the optimized states and inputs from the NMPC.

5. The torque is set as a feed-forward term and is sent to the robot's motor controller. Low-gain joint-space position and velocity **PD commands** are sent to the robot's motors to reduce the shock during foot contact and for better tracking performance.

6. The NMPC and WBC need to know the current robot state, the base orientation, and the joint state, all obtained directly from the IMU and the motors. Running in the same loop with WBC, a **linear Kalman filter** [1]estimates the base position and velocity from base orientation, base acceleration, and joint foot position measurements.

### NMPC
The NMPC part solves the following optimization problems at each cycle through the formulation and solving interfaces
provided by OCS2:

$$
\begin{split}
\begin{cases}
\underset{\mathbf u(.)}{\min} \ \ \phi(\mathbf x(t_I)) + \displaystyle \int_{t_0}^{t_I} l(\mathbf x(t), \mathbf u(t),
t) \, dt \\
\text{s.t.} \ \ \mathbf x(t_0) = \mathbf x_0 \,\hspace{11.5em} \text{initial state} \\
\ \ \ \ \ \dot{\mathbf x}(t) = \mathbf f(\mathbf x(t), \mathbf u(t), t) \hspace{7.6em} \text{system flow map} \\
\ \ \ \ \ \mathbf g_1(\mathbf x(t), \mathbf u(t), t) = \mathbf{0} \hspace{8.2em} \text{state-input equality
constraints} \\
\ \ \ \ \ \mathbf g_2(\mathbf x(t), t) = \mathbf{0} \hspace{10.4em} \text{state-only equality constraints} \\
\ \ \ \ \ \mathbf h(\mathbf x(t), \mathbf u(t), t) \geq \mathbf{0} \hspace{8.55em} \text{inequality constraints}
\end{cases}\end{split}
$$

For this framework, we defined system state $\mathbf{x}$ and input $\mathbf{u}$ as:

$$
\begin{equation} \mathbf{x}= [\mathbf{h}_{com}^T, \mathbf{q}_b^T, \mathbf{q}_j^T]^T,
\mathbf{u} = [\mathbf{w}_e^T, \mathbf{v}_j^T]^T \end{equation}
$$

where $\mathbf{h}_{com} \in \mathbb{R}^6$ is the collection of the normalized centroidal momentum,
$\mathbf{q}=[\mathbf{q}_b^T, \mathbf{q}_j^T]^T$ is the generalized coordinate. $\mathbf{w}_e \in \mathbb{R}^{18}$
consists of contact wrenches at five contact points, i.e., four ground 3-DoF reaction forces of the feet and one 6-DoF reaction wrench at the end-effector. $\mathbf{q}_j$ and
$\mathbf{v}_j$ are the joint positions and velocities.
While the cost function is simply the quadratic cost of tracking the error of all states and the input, the system
dynamics uses centroidal dynamics with the following constraints:

- Zero closed foot contact velocities
- Zero open contact wrench
- Swinging foot contact trajectory tracking
- End-effector position tracking
- Joint positions and velocities operational limits
- Robot self-collisions avoidance
- End-effector force inside friction cone
- End-effector force tracking (see [force-tracking](https://github.com/danisotelo/qm_door/tree/force-tracking) branch)

To solve this optimal control problem, a multiple shooting is formulated to transcribe the optimal control problem to a
nonlinear program (NLP) problem, and the NLP problem is solved using Sequential Quadratic Programming (SQP). The QP
subproblem is solved using HPIPM. For more details [2, 3].

### WBC

<div align="center">

| Priority | Type | Task                                   |
|:--------:|:----:|----------------------------------------|
|     0    |  =   | Floating base equations of motion      |
|     0    |  ≥   | Torque limits                          |
|     0    |  ≥   | Friction cone constraints              |
|     0    |  =   | No motion at the contact points        |
|     1    |  =   | Base motion tracking                   |
|     1    |  =   | Swing feet trajectory tracking         |
|     2    |  =   | Arm joint motion tracking              |
|     2    |  =   | Contact force tracking (see [force-tracking](https://github.com/danisotelo/qm_door/tree/force-tracking) branch)                 |

</div>

WBC only considers the current moment. Several tasks are defined in the table above. Each task corresponds to equality or inequality constraints on decision variables. These decision variables are:

$$
\mathbf{x}_{wbc} = [\ddot{\mathbf{q}}^T, \mathbf{w}_e^T]^T
$$

where $\ddot{\mathbf{q}}$ is acceleration of the generalized coordinates. The WBC solves
the QP problem in the null space of the higher priority tasks' linear constraints and tries to minimize the slacking
variables of inequality constraints. This approach can consider the full nonlinear rigid body dynamics and ensure strict
hierarchy results. For more details [4].



## Bugs & Feature Requests

The project is still in the early stages of development and we welcome feedback. Please report bugs and request features using the [Issue Tracker](https://github.com/danisotelo/qm_door/issues).

## To Do

- [ ] Add gripper controller
- [ ] On-run step height and target velocity update
- [ ] Integrate navigation pipeline

## References

This project is built on top of the following repositories:

- **QM Control by Skywoodsz**: NMPC + WBC for legged manipulators. Check it out [here](https://github.com/skywoodsz/qm_control).

- **Legged Control by QiayuanL**: NMPC + WBC for quadruped robots. Visit their project [here](https://github.com/qiayuanl/legged_control).

Some of the papers that have been used as a reference for building this code are:

[1] T. Flayols, A. Del Prete, P. Wensing, A. Mifsud, M. Benallegue, and O. Stasse, *“Experimental evaluation of simple estimators for humanoid robots,”* IEEE-RAS Int. Conf. Humanoid Robot., pp. 889–895, 2017, doi: [10.1109/HUMANOIDS.2017.8246977](https://ieeexplore.ieee.org/document/8246977).

[2] J. P. Sleiman, F. Farshidian, M. V. Minniti, and M. Hutter, *“A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation,”* IEEE Robot. Autom. Lett., vol. 6, no. 3, pp. 4688–4695, 2021, doi: [10.1109/LRA.2021.3068908](https://ieeexplore.ieee.org/document/9387121).

[3] R. Grandia, F. Jenelten, S. Yang, F. Farshidian, and M. Hutter, *“Perceptive Locomotion through Nonlinear Model Predictive Control,”* (submitted to) IEEE Trans. Robot., no. August, 2022, doi: [10.48550/arXiv.2208.08373]().

[4] C. Dario Bellicoso, C. Gehring, J. Hwangbo, P. Fankhauser, and M. Hutter, *“Perception-less terrain adaptation through whole body control and hierarchical optimization,”* in IEEE-RAS International Conference on Humanoid Robots, 2016, pp. 558–564, doi: [10.1109/HUMANOIDS.2016.7803330]().
