# asar_hybrid_tmp

## Overview

This repository houses action servers controlling two 7DoF manipulators and simulation environments dedicated to suturing tasks. This work is an integral part of a research paper entitled "A Hybrid Framework Combining Task-Motion Planning and Dynamic Behavior Trees for Minimally Invasive Surgery".

![Multi-Throw Suturing Example](gifs/dbt.gif)

## Examples of Suturing Tasks
![Dynamic Behavior Tree (DBT) trajectory adjustment when noisy grasp is applied after grasping](gifs/dbt_no_handover.gif)

![DBT grasp pose and trajectory adjustment when noise is added after grasping ](gifs/dbt_single_handover.gif)

### Dependencies

This project relies on multiple packages for the operation of the 7DoF manipulator, the forceps, and the simulation environment:

- `asar_robot_control_packages` - https://github.com/husikl/asar_robot_control_packages.git 
- `dbt_ros` - https://github.com/husikl/dbt_ros.git
- CoppeliaSim v4.3.0 or newer
- OMPL with python bindings
- PddlStream library - https://github.com/caelan/pddlstream.git
- Pinocchio library built with FCL. To build Pinocchio with FCL, execute the following command:

        ```bash
        cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_PYTHON_INTERFACE=ON -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON
        ```

## Installation
(TO-DO section)

## Usage:
1. Run a CoppeliaSim environment from the sim folder, e.g. stitch_mts_bm2.ttt. This includes two PSMS attached to RCMs, a tissue phantom, and a surgical needle. Feel free to adjust the RCM locations.
2. To initialize a single PSM (robotic arm with surgical tool attached), run:
    - `roslaunch asar_hybrid_tmp asar_test_ompl.launch`
3. For two PSM case, run:
    - `roslaunch asar_hybrid_tmp asar_test_ompl_arm2.launch`
   
The above commands initialize the arms, assuming the simulation is already running.

To set the RCM, adjust the values for: 
    - `<arg name="trocar_x" value="0.4205"/> <arg name="trocar_y" value="0.250"/> <arg name="trocar_z" value="0.201"/>`. 
Note that the frame of reference for both arms is with respect to each arm's base link.

Once the arms are initialized, you can send a goal target to each arm by sending a goal to the action server using:
- `rostopic pub /unit0/arm_grasp_server/goal`. Press tab after this and adjust the command field in the resulting full message type, e.g., 
    `command : 'grasp'`. 
    This will trigger the grasp sampling and motion planning to grasp the needle.

4. Running the benchmark