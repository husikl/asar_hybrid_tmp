cmake_minimum_required(VERSION 3.0.2)
project(asar_hybrid_tmp)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED
  std_msgs
  roscpp
  actionlib_msgs
  actionlib
  # need asar_control package!
  asar_control
)

## System dependencies are found with CMake's conventions
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
find_package(pinocchio  REQUIRED)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BenchmarkData.msg
  ArmSuturePointId.msg
  PddlAction.msg
  GraspTrajectory.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  isPoseReachable.srv
  getPath.srv
  collisionCheck.srv
  isHolding.srv
  getGraspSamples.srv
  getObjectPose.srv
  SuturePoints.srv
  SuturePath.srv
  isSutureIKFeasible.srv
  PddlMotions.srv
  PlanWithPddl.srv
  checkIfPartnerCanGrasp.srv
  AddNoise.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  getTraj.action
  Grasp.action
  PathFollower.action
  
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  geometry_msgs
)


catkin_package(
 INCLUDE_DIRS 
 LIBRARIES
 CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
)


add_executable(asar_ompl_grasp_server
  src/asar_ompl_grasp_server.cpp
)


target_link_libraries(asar_ompl_grasp_server
  ${catkin_LIBRARIES}  
  pinocchio::pinocchio 
)

catkin_install_python(PROGRAMS
  scripts/collision_checker.py
  scripts/ompl_planner.py
  scripts/suture_points_handler.py
  scripts/suture_benchmark.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
