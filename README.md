
Simulation of dual-arm walking-assistive robot based on ROS.

## INSTALLATION

```
sudo apt-get update
sudo apt-get install ros-noetic-pinocchio
sudo apt-get install ros-noetic-ur-description
git clone https://github.com/Lipeng-Robotics/tencent_zju_joint_project.git
cd src/pinocchio_sim_ws
catkin clean
catkin build
```

## DEPENDECIES

ROS-noetic : Robot Operating System https://www.ros.org/

OSQP : 

```
cd osqp
mkdir build
cd build
cmake .. -DBUILD_SHARED_LIBS=ON
make -j6
sudo make install

```

OSQP - EIGEN : 

```
cd osqp-eigen
mkdir build && cd build
cmake ../	#默认安装在/usr/local/include中,非apt安装的包，不在/usr/include/下，而是/usr/local/include下
make
sudo make install
```

可能会遇到找不到osqp.h的问题，参考 [https://blog.csdn.net/superem_/article/details/122705374?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522172109729616800172530264%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&amp;request_id=172109729616800172530264&amp;biz_id=0&amp;utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-2-122705374-null-null.142^v100^pc_search_result_base6&amp;utm_term=%23include%20%3Cosqp%2Fauxil.h%3E%20%23include%20%3Cosqp%2Fscaling.h%3E&amp;spm=1018.2226.3001.4187]()

Sophus FMT  安装比较简单，参考网络即可

## HOW TO START

```
catkin build 
source ./devel/setup.bash
roslaunch dual_arm gazebo.launch
roslaunch tactile_compliance dynacmics.launch
```

If you have realsense D400 series, you can also try the intent recognization node, More details can be found in /src/realsense/README.md.

## MODULE

### 1. Robot Description Simulation Module (/src/dual_arm)

Robot description files for simulation, including SRDF, URDF, and rigid body models.

### 2. Matlab Sim Module (/src/matlab_sim)

Some MATLAB code convenient for simulation, including calculating Jacobian matrix and adaptive whole-body compliant control.

### 3. Optimal Trajectory Generation Module (/src/optimal_traj) (not use)

Generating an optimal trajectory to provide comfortable walking aid, waiting for update

### 4. Realsense D455 Module (/src/realsense)

Get the user's intention and user's body status by realsense D455 and mediapipe.

### 5. Whole-body Adaptive Admittance Control Module (/src/tactile_compliance)

A whole-body adaptive admittance control strategy for whole-body compliance
