Simulation of dual-arm walking-assistive robot based on ROS.

## INSTALLATION

```
sudo apt-get update
sudo apt-get install ros-noetic-pinocchio
sudo apt-get install ros-noetic-ur-description
git clone https://github.com/sunyu-robot/Tactile-Sevice-Robot-Mobility-Assistance.git
cd Tactile-Sevice-Robot-Mobility-Assistance/pinocchio_sim_ws
catkin clean
```

## DEPENDECIES

ROS-noetic : Robot Operating System https://www.ros.org/

OSQP0.6.0 : https://github.com/osqp/osqp/

OSQP - EIGEN 0.7.0 : https://github.com/robotology/osqp-eigen

Sophus1.x : https://github.com/strasdat/Sophus

FMT-8.1.1 : https://github.com/fmtlib/fmt

Ipopt : https://github.com/coin-or/Ipopt

CasADi : https://github.com/casadi/casadi

## HOW TO START

```
catkin build util_msgs
catkin build
source ./devel/setup.bash
roslaunch dual_arm gazebo.launch
roslaunch tactile_compliance dynacmics.launch
```

## MODULE

### 1. Robot Description Simulation Module (/src/dual_arm)

Robot description files for simulation, including SRDF, URDF, and rigid body models.

### 2. Optimal Trajectory Generation Module (/src/optimal_traj)

Generating an optimal trajectory to provide comfortable walking aid, waiting for update

### 3. Whole-body Adaptive Admittance Control Module (/src/tactile_compliance)

A whole-body adaptive admittance control strategy for whole-body compliance

### 4. Message Types(/src/util_msgs)

ROS message types

## PREVIEW

![anim](robot.gif)
