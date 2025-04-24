/**
 * @file MainController.hpp
 * @brief head file of main program
 * @author Yu Sun,Cong Xiao
 * @maintainer Lipeng Chen
 * @version 0.1.0
 * @date 10.28 2023
 */

#pragma once
// C++ standard headers
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <algorithm>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Geometry>
#include <std_msgs/UInt8.h>
// osqp
#include "OsqpEigen/OsqpEigen.h"
// other head file in the pkg
#include "TactileCompliance.hpp"
#include "CollisionConstraint.hpp"
#include "CriticalPoint.hpp"
#include "Utils.hpp"

#define LOOP_RATE 100.0

using namespace BodyMath;

class Tactile_HQP
{
public:
    CollisionConstraint collision_constraint;
    TactileCompliance tactile_compliance;
    CriticalPoint critical_point;

    Tactile_HQP() : loop_rate(LOOP_RATE)
    {
        init();
    }

    ~Tactile_HQP() = default;
    /**
    * @brief Init function
    * @detail
    *  Init params ,publishers and subscribers ,load the urdf model
    */
    void init();

    /**
    * @brief Control loop
    * @detail
    *  Control loop
    */
    void spin();

    /**
    * @brief Joints' states callback
    * @detail
    *  Get the joints' states of Dual-arm robot from the joint state topic 
    * @param[in] &input        joints' states with stamp
    */
    void jointstateCallback(const sensor_msgs::JointStateConstPtr &input); 

    /**
    * @brief Robot return to init pose
    * @detail
    *  Robot return to init pose with Kp tracking control
    */
    void robotInitPose(const Vector15d& input);

    /**
    * @brief visualize the marker
    * @detail
    *  marker visualization
    */
    void markerVisualization();

    /**
    * @brief publish the command to robot
    * @detail
    *  publish data to all joint
    */
    void publishCommand();

    /**
    * @brief get q cmd
    * @detail
    *  compute q command by HQP
    */
   void getCommand();

public:
    // robot kinematics
    // node handle
    ros::NodeHandle nh;
    // Loop rate
    ros::Rate loop_rate;
    // Loop time
    double loop_time;
    // joint state subscriber
    ros::Subscriber joint_state_sub;
    // marker pub
    ros::Publisher marker_pub;
    // publisher velocity array
    std::array<std_msgs::Float64, 15> dq_to_pub;
    std::array<ros::Publisher, 15> dq_pub;
    // init joint pose
    Vector15d init_joints_pose;
    // Interval between actions
    double timer;
    // joint position limit
    vector<double> q_limit;
    // velocity limit
    double velocity_limit;
    // joint velocity command
    Vector15d dq_cmd;
    // sum time once a looprate
    double sum_time, count;
    // real time joint space position and velocity
    Vector15d q_real, dq_real;
    // marker array
    visualization_msgs::MarkerArray marker_array;
    // expansion of robot point and obstacle point
    sphere point_sphere[35];

    // HQP desired matrix
    // OSQP solver
    OsqpEigen::Solver solver;
    // hessian
    Eigen::SparseMatrix<double> hessian_first;
    Eigen::SparseMatrix<double> hessian_second;
    // constraint matrix
    Eigen::SparseMatrix<double> LinearConstraintsMatrix;
    // gradient
    Eigen::VectorXd gradient_first;
    Eigen::VectorXd gradient_second;
    // constraint bound
    Matrix<double, 815, 1> lowerBound;
    Matrix<double, 815, 1> upperBound;
    // intermediate variable for OSQP
    Matrix15d Heq;
    Vector15d gradient_vector;
    Matrix15d hessian;
    // Identity matrix
    Matrix15d I15d;
    // solution of OSQP
    Vector15d dq_solution_first, dq_solution_second;
};