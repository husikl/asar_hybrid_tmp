# asar_hybrid_tmp

## Overview

This repository contains action servers to control two 7DoF manipulators and simulation environments for suturing tasks. This work is part of a research paper: "A Hybrid Framework Combining Task-Motion Planning and Dynamic Behavior Trees for Minimum Invasive Surgery".

![Multi-Throw Suturing Example](gifs/dbt_no_handover.gif)

## Installation

### Dependencies

This project depends on several packages for controlling the 7DoF manipulator, the forceps, and the simulation environment:

- `asar_control`
- `asar_description`
- `codcs_ik`
- `gen3_control`
- `forceps_control`
- `mc_daq_ros`
- `virtuose_ros`
- CoppeliaSim
- OMPL with python bindings
- Pinnochio library built with FCL:

To build Pinocchio with FCL, use the following command:

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DPYTHON_EXECUTABLE=/usr/bin/python3 -DBUILD_PYTHON_INTERFACE=ON -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON

## Examples
![Dynamic Behavior Tree (DBT) trajectory adjustment when noisy grasp is applied after grasping](gifs/dbt_no_handover.gif)

![DBT trajectory adjustment when noisy grasp is applied after grasping, example 2](gifs/dbt.gif)

![DBT grasp pose and trajectory adjustment when noise is added after grasping ](gifs/dbt_single_handover.gif)
## Usage
Provide instructions on how to use your project. Include plenty of examples.

## Contributing
Provide instructions on how contributors can help improve your project. If you have specific tasks that need to be tackled, list them here.

## License
Include information about your project's license here.