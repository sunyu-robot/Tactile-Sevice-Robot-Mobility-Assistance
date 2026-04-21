/**
 * @file MainController.hpp
 * @brief HQP-based dual-arm tactile compliance controller
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 0.1.0
 * @date 10.28 2023
 */
#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
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
#include "OsqpEigen/OsqpEigen.h"
#include "TactileCompliance.hpp"
#include "CollisionConstraint.hpp"
#include "CriticalPoint.hpp"
#include "Utils.hpp"

#define LOOP_RATE 100.0  // Hz

using namespace BodyMath;

class Tactile_HQP
{
public:
    CollisionConstraint collision_constraint;
    TactileCompliance   tactile_compliance;
    CriticalPoint       critical_point;

    Tactile_HQP() : loop_rate(LOOP_RATE) { init(); }
    ~Tactile_HQP() = default;

    // Initialize ROS interfaces, load parameters, set up OSQP, and move to home pose.
    void init();

    // Main control loop at LOOP_RATE Hz.
    void spin();

    // ROS callback: unpack /joint_states into q_real and dq_real (15-DOF order).
    void jointstateCallback(const sensor_msgs::JointStateConstPtr &input);

    // Drive robot to target joint pose using proportional velocity control.
    void robotInitPose(const Vector15d& input);

    // Publish collision sphere and tactile cylinder markers to RViz.
    void markerVisualization();

    // Publish dq_cmd to all 15 joint velocity controllers.
    void publishCommand();

    /**
     * @brief Solve the two-layer HQP and write the result into dq_cmd.
     *
     * Layer 1: tactile position tracking (active).
     * Layer 2: end-effector rotation tracking (currently disabled).
     * Collision avoidance CBF constraints are injected into the OSQP problem.
     */
    void getCommand();

public:
    ros::NodeHandle nh;
    ros::Rate       loop_rate;
    double          loop_time;
    ros::Subscriber joint_state_sub;
    ros::Publisher  marker_pub;
    std::array<std_msgs::Float64, 15> dq_to_pub;
    std::array<ros::Publisher,    15> dq_pub;

    Vector15d init_joints_pose;
    double    timer;           // timestamp of last joint state message [s]
    vector<double> q_limit;
    double    velocity_limit;
    Vector15d dq_cmd;
    double    sum_time, count; // for average loop-time logging
    double    qp_solve_time_ms_;
    double    qp_sum_ms_, qp_count_;  // accumulate during a tracking event for avg QP time
    std::string perf_log_path_;
    Vector15d q_real, dq_real;
    visualization_msgs::MarkerArray marker_array;
    sphere    point_sphere[35];

    // ── OSQP problem data ─────────────────────────────────────────────────────
    OsqpEigen::Solver solver;
    // 815 constraints = 800 CBF rows + 15 joint-limit rows
    Eigen::SparseMatrix<double>  hessian_first, hessian_second;
    Eigen::SparseMatrix<double>  LinearConstraintsMatrix;
    Eigen::VectorXd              gradient_first, gradient_second;
    Matrix<double, 815, 1>       lowerBound, upperBound;
    Matrix15d  Heq;             // dense Hessian assembled by tactileTrack
    Vector15d  gradient_vector;
    Matrix15d  hessian;         // Heq + regularisation
    Matrix15d  I15d;
    Vector15d  dq_solution_first, dq_solution_second;
};
