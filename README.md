# asar_hybrid_tmp

## Overview
This repository implements concepts from the research paper: "Toward Autonomous Robotic Minimally Invasive Surgery: A Hybrid Framework Combining Task-Motion Planning and Dynamic Behavior Trees."
It combines a task-motion planner with dynamic behavior trees to enable more robust and autonomous robotic surgical procedures.


![Multi-Throw Suturing Example](gifs/dbt.gif)

## Examples of Suturing Tasks
![Dynamic Behavior Tree (DBT) trajectory adjustment when noisy grasp is applied after grasping](gifs/dbt_no_handover.gif)

![DBT grasp pose and trajectory adjustment when noise is added after grasping ](gifs/dbt_single_handover.gif)

### Dependencies

This project depends on several packages for controlling the 7DoF manipulator, the forceps, and the simulation environment:
- Python3 for ROS Noetic (Ubuntu 20.04)
- `asar_robot_control_packages` - https://github.com/husikl/asar_robot_control_packages.git 
- `dbt_ros` - https://github.com/husikl/dbt_ros.git
- CoppeliaSim v4.3.0 or later
- OMPL with Python bindings
- PddlStream library - https://github.com/caelan/pddlstream.git
- Pinocchio library built with FCL. Execute the following command to build Pinocchio with FCL:

        ```bash
        cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_PYTHON_INTERFACE=ON -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON
        ```

## Installation
(TO-DO section)

## Usage

1. Launch a CoppeliaSim environment from the sim folder (e.g., stitch_mts_bm2.ttt). This includes two PSMS attached to RCMs, a tissue phantom, and a surgical needle. Feel free to adjust the RCM locations.
2. To initialize a single PSM (robotic arm with surgical tool attached), run `roslaunch asar_hybrid_tmp asar_test_ompl.launch`.
3. For the two PSM case, run `roslaunch asar_hybrid_tmp asar_test_ompl_arm2.launch`. The above commands initialize the arms, assuming the simulation is already running.
4. To set the RCM, adjust the values for: `<arg name="trocar_x" value="0.4205"/> <arg name="trocar_y" value="0.250"/> <arg name="trocar_z" value="0.201"/>`. Note that the frame of reference for both arms is with respect to each arm's base link.
5. After initializing the arms, you can send a goal target to each arm by sending a goal to the action server: `rostopic pub /unit0/arm_grasp_server/goal`. Press tab after this and adjust the command field in the resulting full message type (e.g., `command : 'grasp'`). This will trigger the grasp sampling and motion planning to grasp the needle.

To run TMP with pddlstream (assumes that pddlstream is built), follow these steps:

1. Navigate to the pddlstream directory and copy the folder `tmp_suturing` from this repo to "/pddlstream_directory/examples/".
2. From the root directory for pddlstream, you can initialize the ROS action service for the TMP planner by running `python -m examples.tmp_suturing.run` in a new terminal.

To run the DBT planner and executor, use the following commands in two separate terminals (assuming that dbt_ros is built):

1. To run the executor node (which waits for BT and executes it once received), enter `rosrun dbt_ros bt_executor`.
2. To run the DBT tree generator, enter `rosrun dbt_ros pddl_to_bt_service.py`.

To run some of the suturing benchmarks, select the function in `suture_benchmark.py`. For example, to use `def benchmark_loop(self,)` on line 864, comment or uncomment the function to benchmark the motion and run `rosrun asar_hybrid_tmp suture_benchmark.py`.

To start the benchmark in a new terminal, use the command `rosservice call /start_benchmark "{}"`.

#### TODO: Combine nodes to run the benchmark and create JSON-based config selection to run different tasks.

## Reference
If you find this repository helpful for your research, please consider citing our paper:
```bibtex
@ARTICLE{fozilov_2023,

  author={Fozilov, Khusniddin and Colan, Jacinto and Sekiyama, Kosuke and Hasegawa, Yasuhisa},

  journal={IEEE Access}, 

  title={Toward Autonomous Robotic Minimally Invasive Surgery: A Hybrid Framework Combining Task-Motion Planning and Dynamic Behavior Trees}, 

  year={2023},

  volume={11},

  number={},

  pages={91206-91224},

  keywords={Surgery;Planning;Robots;Needles;Robot kinematics;Trajectory;Autonomous systems;Minimally invasive surgery;Medical robotics;Motion planning;Autonomous systems;behavior trees;hierarchical deliberation;minimally invasive surgery;multi-throw suturing;nonlinear optimization;robotic surgery;task and motion planning},

  doi={10.1109/ACCESS.2023.3308619}}
```
