/**
 * @file TactileCompliance.hpp
 * @brief Admittance-based tactile compliance controller for dual-arm robot
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 10.28 2023
 */
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include "Utils.hpp"
#include <visualization_msgs/MarkerArray.h>
#include "OsqpEigen/OsqpEigen.h"
#include <util_msgs/trajectory.h>
#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include <sophus/se3.hpp>

using namespace BodyMath;

class TactileCompliance
{
public:
    TactileCompliance();
    ~TactileCompliance() = default;

    // Linear Jacobian: J_lin(joint_id, q) → 3×15, one entry per contact point (ls/rs/lb/rb).
    Matrix<double, 3, 15> (*linearJacobianFuncArray[4])(int, double, double, double, double, double, double, double) =
        {getJacobiantouch_ls, getJacobiantouch_rs, getJacobiantouch_lb, getJacobiantouch_rb};

    // Angular Jacobian: J_ang(q) → 3×15, one entry per contact point.
    Matrix<double, 3, 15> (*angularJacobianFuncArray[4])(double, double, double, double, double, double, double) =
        {getJacobianrotate_ls, getJacobianrotate_rs, getJacobianrotate_lb, getJacobianrotate_rb};

    // Forward kinematics: contact origin position in world frame.
    Vector3d (*jointPosFuncArray[4])(double, double, double, double, double, double, double, double) =
        {getpoint_ls, getpoint_rs, getpoint_lb, getpoint_rb};

    // End-effector rotation matrix R(q).
    Matrix3d (*rotationFuncArray[4])(double, double, double, double) =
        {getrotate_ls, getrotate_rs, getrotate_lb, getrotate_rb};

    /**
     * @brief Assemble the QP Hessian and gradient for tactile position tracking (HQP layer 1).
     *
     * Runs admittance dynamics for each contact point, computes the desired Cartesian
     * velocity dx_track, then accumulates:
     *   Heq          += w * J_lin^T * J_lin
     *   gradient     += -w * dx_track^T * J_lin
     * Also accumulates rotation tracking terms with a small weight (Q_rotate).
     *
     * @param q_real         Current joint positions (15×1).
     * @param dq_real        Current joint velocities (15×1).
     * @param gradient_vector  Output gradient (15×1).
     * @param Heq            Output Hessian (15×15).
     * @param dq_cmd         Output joint velocity command (15×1, set to zero here; filled by OSQP).
     */
    void tactileTrack(const Vector15d q_real, const Vector15d dq_real,
                      Vector15d& gradient_vector, Matrix15d& Heq, Vector15d& dq_cmd);

    // Populate marker_array with cylinder (contact patch) and arrow (force) markers.
    void tactileMarker(visualization_msgs::MarkerArray &marker_array);

    // Set admittance parameters Mxinv, Bx, Kx and the control loop time.
    void setInitVariable(const Matrix3d _Mxinv, const Matrix3d _Bx, const Matrix3d _Kx, const double _loop_time);

    /**
     * @brief Assemble QP data for rotation tracking (HQP layer 2). Currently unused.
     *
     * Constrains the layer-2 solution to preserve the layer-1 Cartesian velocity,
     * then minimises angular velocity error.
     */
    void tactileRotation(const Vector15d dq_solution_first,
                         Vector15d& gradient_vector, Matrix15d& Heq,
                         Eigen::SparseMatrix<double> &LinearConstraintsMatrix,
                         Matrix<double, 815, 1> &lowerBound,
                         Matrix<double, 815, 1> &upperBound);

    // ROS callback: receive optimal forces and desired contact positions from the NMPC planner.
    void trajectoryCallback(const util_msgs::trajectoryConstPtr &input);

    // ROS callback: update human CoM pose and rotation matrix.
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &input);

    // Publish current left/right contact positions to /contact_position_{left,right}.
    void publishContact();

    // SO(3) exponential map: axis-angle vector → rotation matrix.
    Eigen::Matrix3d expMap(const Eigen::Vector3d& omega);

    // SO(3) logarithmic map: rotation matrix → axis-angle vector.
    Eigen::Vector3d logMap(const Eigen::Matrix3d& rotation_matrix);

    // ── Tracking lag measurement (read by MainController) ─────────────────────
    bool     tracking_active_;
    bool     tracking_moving_;   // true once contact velocity exceeded 0.02 m/s after trigger
    std::chrono::steady_clock::time_point tracking_start_;
    Vector3d dp_left_ref_, dp_right_ref_;
    Matrix3d last_logged_rotation_;
    double   last_lag_ms_;
    std::string perf_log_path_;

private:
    ros::NodeHandle nh;
    ros::Publisher  contact_left_pub, contact_right_pub;
    geometry_msgs::Vector3 contact_position_left_msg, contact_position_right_msg;

    double   sum_weight;
    double   loop_time;
    // Admittance parameters for linear motion: Mx⁻¹, Bx, Kx
    Matrix3d Mxinv, Bx, Kx;
    // Admittance parameters for rotational motion
    Matrix3d Mxinv_rotate, Bx_rotate, Kx_rotate;

    ros::Time     start_time, current_time;
    ros::Duration elapsed_time;
    double        elapsed_sec;  // seconds since first contact initialisation

    ros::Subscriber userpose_sub;
    Matrix3d human_rotation_matrix;
    Vector3d human_position;

    Eigen::Matrix<double, -1, 15> constriant_matrix;
    Eigen::Matrix<double, 815, 1> upper_bound_tactile, lower_bound_tactile;

    Vector3d sum_delta_zmp;
    double   Kp, Kd;  // position/velocity gains for dx_track computation

    util_msgs::trajectory trajectory_msg;
    ros::Subscriber trajectory_sub;
    Vector3d force_left, force_right;  // desired contact forces from NMPC (world frame after rotation)
    Vector3d dp_left,    dp_right;     // desired contact positions from NMPC (world frame)
    Vector3d dp_left_accu, dp_right_accu;

    struct skinstate {
        bool     intouch;
        // Admittance state for linear motion
        Vector3d ddx_error, dx_error, x_error;
        Eigen::Matrix<double, 3, 15> jacobian_matrix_linear;
        double   weight;
        int      joint_id;  // tactile cell column index, maps to forearm joint
        Matrix3d rotation_matrix;
        Matrix3d rotation_matrix_init;
        Vector3d start_position_init;
        uint16_t num_pixels;
        Vector6d wrench;
        Vector3d force;
        Vector3d force_hip;  // moment arm from joint origin to ZMP
        // Desired trajectory for the contact point
        Vector3d ddx_desire_skin, dx_desire_skin, x_desire_skin;
        // Admittance-modified desired trajectory
        Vector3d ddx_newdesire_skin, dx_newdesire_skin, x_newdesire_skin;
        Vector3d dx_real_skin, x_real_skin;
        Vector3d x_track;   // position tracking error + desired velocity
        Vector3d dx_track;  // velocity command = Kp*x_track - Kd*dx_real
        // ZMP vectors
        Vector3d zmp_vector;        // ZMP offset in world frame
        Vector3d zmp_vector_local;  // ZMP offset in link frame
        Vector3d zmp_vector_last;
        Vector3d delta_zmp_vector;
        visualization_msgs::Marker cylinder;
        // Admittance state for rotational motion
        Eigen::Matrix<double, 3, 15> jacobian_matrix_angular;
        Matrix3d x_error_rotation_matrix;
        Sophus::SO3d so3_rotation;          // incremental rotation from admittance
        Sophus::SO3d so3_rotation_current;  // accumulated rotation error
        Sophus::SO3d so3_rotation_next;
        Vector3d so3_log;   // log(R_track) used in gradient
        Vector3d torque;
        Vector3d ddx_error_rotation, dx_error_rotation, x_error_rotation;
        Matrix3d rotation_matrix_d;      // desired rotation (from human pose)
        Matrix3d rotation_matrix_d_new;  // desired rotation after admittance correction
        Matrix3d track_rotation_matrix;  // R_d_new * R_current^T
        // Desired ZMP position tracking
        Vector3d desire_zmp_world;
        Vector3d desire_zmp_local;
        Vector3d x_zc;              // offset: current ZMP - desired ZMP
        Vector3d x_desire_skin_zmp; // desired contact pos at initial ZMP
        // RViz force arrow
        visualization_msgs::Marker arrow;
        Vector3d start_position;
        geometry_msgs::Point   start_point, end_point;
        geometry_msgs::Vector3 direction;
        bool     initialized;
        Matrix3d init_error_rotation;
    } skinstates[4];
};
