/**
 * @file TactileCompliance.hpp
 * @brief head file of tactile compliance
 * @author Yu Sun,Cong Xiao
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
#include "Utils.hpp"
#include <visualization_msgs/MarkerArray.h>
#include "OsqpEigen/OsqpEigen.h"
#include <util_msgs/trajectory.h>
// sophus
#define FMT_HEADER_ONLY
#include "fmt/format.h"
#include <sophus/se3.hpp>

using namespace BodyMath;

class TactileCompliance
{
public:
    // Constructor
    TactileCompliance();
    // Destructor
    ~TactileCompliance() = default;
    // jacobian func array (linear)
    Matrix<double, 3, 15> (*linearJacobianFuncArray[4])(int, double, double, double, double, double, double, double) = {getJacobiantouch_ls, getJacobiantouch_rs, getJacobiantouch_lb, getJacobiantouch_rb};
    // jacobian func array (angular)
    Matrix<double, 3, 15> (*angularJacobianFuncArray[4])(double, double, double, double, double, double, double) = {getJacobianrotate_ls, getJacobianrotate_rs, getJacobianrotate_lb, getJacobianrotate_rb};
    // real positon func array
    Vector3d (*jointPosFuncArray[4])(double, double, double, double, double, double, double, double) = {getpoint_ls, getpoint_rs, getpoint_lb, getpoint_rb};
    // rotation func array
    Matrix3d (*rotationFuncArray[4])(double, double, double, double) = {getrotate_ls, getrotate_rs, getrotate_lb, getrotate_rb};


    void tactileTrack(const Vector15d q_real, const Vector15d dq_real, Vector15d& gradient_vector, Matrix15d& Heq, Vector15d& dq_cmd);

    void tactileMarker(visualization_msgs::MarkerArray &marker_array);

    void setInitVariable(const Matrix3d _Mxinv, const Matrix3d _Bx, const Matrix3d _Kx, const double _loop_time);

    void tactileRotation(const Vector15d dq_solution_first, Vector15d& gradient_vector, Matrix15d& Heq, Eigen::SparseMatrix<double> &LinearConstraintsMatrix,Matrix<double, 815, 1> &lowerBound,Matrix<double, 815, 1> &upperBound);

    void trajectoryCallback(const util_msgs::trajectoryConstPtr &input); 
    
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &input);

    void publishContact();

    // /**
    // * @brief User pose callback
    // * @detail
    // *  User pose callback
    // */
    // void poseCallback(const geometry_msgs::PoseStampedConstPtr &input);

    Eigen::Matrix3d expMap(const Eigen::Vector3d& omega);

    Eigen::Vector3d logMap(const Eigen::Matrix3d& rotation_matrix);
private:
    // ros nodehandle
    ros::NodeHandle nh;
    // contact position pub
    ros::Publisher contact_left_pub, contact_right_pub;
    // contact position msg
    geometry_msgs::Vector3 contact_position_left_msg, contact_position_right_msg;

    // sum weight of each links
    double sum_weight;
    // loop time 
    double loop_time;
    // cartesian admittance inertia, damping and stiff matrix (linear)
    Matrix3d Mxinv,Bx,Kx;
    // cartesian admittance inertia, damping and stiff matrix (rotation)
    Matrix3d Mxinv_rotate, Bx_rotate, Kx_rotate;
    // start time
    ros::Time start_time;
    // current time
    ros::Time current_time;
    // duration
    ros::Duration elapsed_time;
    // userpose Subscriber
    ros::Subscriber userpose_sub;
    // huamn rotation matrix
    Matrix3d human_rotation_matrix;
    // human COM position (world frame)
    Vector3d human_position;
    // duration second type
    double elapsed_sec;
    // constraint matrix
    Eigen::Matrix<double, -1, 15> constriant_matrix;
    // cbf vector
    Eigen::Matrix<double, 815, 1> upper_bound_tactile;
    Eigen::Matrix<double, 815, 1> lower_bound_tactile;

    Vector3d sum_delta_zmp;
    // Kp  Kd
    double Kp, Kd;
    // trajectory
    util_msgs::trajectory trajectory_msg;
    // trajectory subscriber
    ros::Subscriber trajectory_sub;
    // force in contact
    Vector3d force_left, force_right;
    // contact position
    Vector3d dp_left, dp_right;
    // contact position accumulate
    Vector3d dp_left_accu, dp_right_accu;

    struct skinstate{
    // true means the skin is touching
    bool intouch;
    // x deflection in admittance control
    Vector3d ddx_error, dx_error, x_error;
    // jacobian matrix of each skin
    Eigen::Matrix<double,3,15> jacobian_matrix_linear;
    // weight in QP
    double weight;
    // joint id
    int joint_id;
    // rotation matrix
    Matrix3d rotation_matrix;
    // rotation matrix (init)
    Matrix3d rotation_matrix_init;
    // start pos (init)
    Vector3d start_position_init;
    // numbers of pixels in contact of each skin
    uint16_t num_pixels;
    // 6d wrenchs of each skin
    Vector6d wrench;
    // 3d force of each skin
    Vector3d force;
    // force arm (from joint origin to zero moment point) of each skin
    Vector3d force_hip;
    // task desired trajectory (each touch point)
    Vector3d ddx_desire_skin, dx_desire_skin, x_desire_skin;
    // new task desired trajectory (each touch point)
    Vector3d ddx_newdesire_skin, dx_newdesire_skin, x_newdesire_skin;
    // real time velocity and position
    Vector3d dx_real_skin, x_real_skin;
    // tracking error
    Vector3d x_track;
    // tracking velocity
    Vector3d dx_track;
    // zmp vector (world frame)
    Vector3d zmp_vector;
    // zmp vector (local frame)
    Vector3d zmp_vector_local;
    // zmp last time (world frame)
    Vector3d zmp_vector_last;
    // delta zmp (world frame)
    Vector3d delta_zmp_vector;
    // cylindar marker
    visualization_msgs::Marker cylinder;

    // for rotation
    // jacobian matrix (angular) of each skin
    Matrix<double,3,15> jacobian_matrix_angular;
    // x_error rotation matrix
    Matrix3d x_error_rotation_matrix;
    // so(3) of error rotation matrix
    Sophus::SO3d so3_rotation;
    // so(3) of current x deflection
    Sophus::SO3d so3_rotation_current;
    // so(3) of x deflection of next iteration
    Sophus::SO3d so3_rotation_next;
    // log of so(3)
    Vector3d so3_log;
    // torque
    Vector3d torque;
    // x dx ddx deflection for rotation
    Vector3d ddx_error_rotation, dx_error_rotation, x_error_rotation;
    // desired rotation matrix
    Matrix3d rotation_matrix_d;
    // new desired rotation matrix
    Matrix3d rotation_matrix_d_new;
    // track rotation matrix
    Matrix3d track_rotation_matrix;

    // for desired zmp and desired pos
    // desired zmp (world frame)
    Vector3d desire_zmp_world;
    // desired zmp (local frame)
    Vector3d desire_zmp_local;
    // desired zmp & current zmp transform vector
    Vector3d x_zc;
    // desired traj for desired zmp (by camera or tactile)
    Vector3d x_desire_skin_zmp;
    // force arrow
    // arrow marker
    visualization_msgs::Marker arrow;
    // start joint position
    Vector3d start_position;
    // start point
    geometry_msgs::Point start_point;
    // direction 
    geometry_msgs::Vector3 direction;
    // end point
    geometry_msgs::Point end_point;

    // to judge if the program is initailized
    bool initialized;

    Matrix3d init_error_rotation;
    }skinstates[4];
};