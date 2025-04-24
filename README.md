Simulation of dual-arm walking-assistive robot based on ROS.

## INSTALLATION

```
sudo apt-get update
sudo apt-get install ros-noetic-pinocchio
sudo apt-get install ros-noetic-ur-description
git clone https://github.com/Lipeng-Robotics/tencent_zju_joint_project.git
cd src/pinocchio_sim_ws
catkin clean
```

## DEPENDECIES

ROS-noetic : Robot Operating System https://www.ros.org/

OSQP0.6.0 :

OSQP - EIGEN 0.7.0:

可能会遇到找不到osqp.h的问题，参考 [https://blog.csdn.net/superem_/article/details/122705374?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522172109729616800172530264%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&amp;request_id=172109729616800172530264&amp;biz_id=0&amp;utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-122705374-null-null.142^v100^pc_search_result_base6&amp;utm_term=%23include%20%3Cosqp%2Fauxil.h%3E%20%23include%20%3Cosqp%2Fscaling.h%3E&amp;spm=1018.2226.3001.4187]()

Sophus :

FMT-8.1.1 :

Ipopt:

CasADi:

## HOW TO START

```
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
