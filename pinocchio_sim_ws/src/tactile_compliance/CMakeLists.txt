cmake_minimum_required(VERSION 3.0.2)
project(tactile_compliance)

# SET(CMAKE_BUILD_TYPE "Debug")
# SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
# SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

# set(CMAKE_BUILD_TYPE DEBUG)
if(NOT CMAKE_BUILD_TYPE)
  # set(CMAKE_BUILD_TYPE RelWithDebInfo)
  set(CMAKE_BUILD_TYPE Release)
endif()

# if(USE_UBUNTU_20)
#     message("You are using Ubuntu 20.04, fmt::fmt will be linked")
#     find_package(fmt REQUIRED)
#     set(FMT_LIBRARIES fmt::fmt)
# endif()
# set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall -pthread -fopenmp")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  util_msgs
)

find_package(Eigen3 REQUIRED NO_MODULE)	
find_package(pinocchio REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Sophus REQUIRED)
# find_package(hpp-fcl REQUIRED)
# find_package(yaml-cpp REQUIRED)	

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${FMT_LIBRARIES}
  /usr/local/include/osqp/include
)

link_directories(
  ${catkin_LIB_DIRS}
  /usr/local/include/osqp/lib      
  ...
)

## Declare a C++ library
add_library(tactile_compliance
  src/MainController.cpp
  src/TactileCompliance.cpp
  src/CollisionConstraint.cpp
  src/CriticalPoint.cpp
)

add_executable(init src/init.cpp)

target_link_libraries(init
tactile_compliance 
  ${catkin_LIBRARIES} 
  ${FMT_LIBRARIES}
  Sophus::Sophus
  Eigen3::Eigen
  # pinocchio::pinocchio
  osqp         
  OsqpEigen    
  # hpp-fcl::hpp-fcl
  # /lib/x86_64-linux-gnu/libomp5.so
  # /usr/lib/x86_64-linux-gnu/libyaml-cpp.a
)