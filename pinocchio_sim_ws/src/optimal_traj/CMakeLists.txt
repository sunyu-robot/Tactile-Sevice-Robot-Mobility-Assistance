cmake_minimum_required(VERSION 3.11)
project(optimal_traj)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 17)
set(CASADI_INCLUDE_DIRS /usr/local/include/casadi)
set(CASADI_LIBRARIES /usr/local/lib/libcasadi.so)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  util_msgs
  interactive_markers
)
find_package(Eigen3 REQUIRED NO_MODULE)
catkin_package(
 INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CASADI_INCLUDE_DIRS}
  # ${PINOCCHIO_INCLUDE_DIRS}
  # /src/wholebody_compliance/include
)


add_library(optimal_traj
  src/Trajectory.cpp
  # src/DummyHuman.cpp
)

add_executable(trajectory src/OptimalTrajectoryNode.cpp)

target_link_libraries(trajectory
  optimal_traj 
  ${catkin_LIBRARIES} 
  ${CASADI_LIBRARIES}
  Eigen3::Eigen
)