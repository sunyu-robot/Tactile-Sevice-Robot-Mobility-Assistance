/**
 * @file Trajectory.hpp
 * @brief Centroidal NMPC planner for human mobility assistance
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 04.19 2023
 */
#ifndef optimal_trajectory
#define optimal_trajectory

#include <float.h>
#include <ros/ros.h>
#include <math.h>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <util_msgs/trajectory.h>
#include <casadi/casadi.hpp>
#include <cmath>
#include "visualization_msgs/Marker.h"
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <fstream>
#include <chrono>

#define PLANNING_RATE   10.0   // Hz
#define SLIP_MAX_SPEED   2.0   // m/s, max tactile contact sliding speed

using namespace Eigen;

typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 6, 1>  Vector6d;
typedef Matrix<double, 6, 6>  Matrix6d;

class OptimalTrajectory
{
public:
    OptimalTrajectory() : loop_rate(PLANNING_RATE),
                          inertial_matrix(3, 3), inertial_matrix_inv(3, 3),
                          L_foot_com(3, 1),
                          Q_casadi(18, 18), R_casadi(12, 12),
                          X_ref(18, 1), q0(18, 1)
    {
        init();
    }

    // Initialize ROS interfaces, model parameters, and build the NMPC problem.
    void init();

    // Main control loop: blocks until ROS shuts down.
    void spin();

    // ROS callback: update human CoM pose and ZYX Euler angles from odometry.
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &input);

    // ROS callback: update left tactile contact position (world frame).
    void tactileLeftCallback(const geometry_msgs::Vector3ConstPtr &input);

    // ROS callback: update right tactile contact position (world frame).
    void tactileRightCallback(const geometry_msgs::Vector3ConstPtr &input);

    // Publish optimal forces and desired tactile positions to /trajectory.
    void publishCommand();

    // Assemble current state vector q0 and reference X_ref from latest sensor data.
    void getHumanPose();

    // Wrap angle to [-pi, pi].
    inline double constrainAngle(double angle)
    {
        angle = fmod(angle, 2 * M_PI);
        if (angle > M_PI)       angle -= 2 * M_PI;
        else if (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    // First-order low-pass filter: y = alpha*last + (1-alpha)*current.
    inline double lowpassFilter(double last, double current, double alpha = 0.9)
    {
        return alpha * last + (1.0 - alpha) * current;
    }

    /**
     * @brief Solve the centroidal NMPC for one planning step.
     *
     * Updates q0_param_ and X_ref_param_ then calls the pre-built IPOPT solver.
     * Writes optimal forces (force_left/right) and contact velocity commands
     * (dpos_left/right) into member variables.
     */
    void centroidalNMPC();

    /**
     * @brief Centroidal dynamics: xdot = f(x, u).
     *
     * State x (18×1): ZYX Euler angles, CoM position, angular velocity (body),
     *                 CoM velocity (body), left/right contact positions (body).
     * Input u (12×1): left/right contact forces f1,f2 and contact velocities dp1,dp2
     *                 (all in body frame).
     */
    casadi::MX centroidalDynamics(const casadi::MX& current_state, const casadi::MX& input);

    /**
     * @brief Ground reaction force model (body frame).
     *
     * Linearised normal force derived from the inverted-pendulum ground contact.
     * @param euler_angle  ZYX Euler angles of the human trunk (3×1).
     * @param omega        Angular velocity in body frame (3×1).
     * @return             Ground reaction force N in body frame (3×1).
     */
    inline casadi::MX groundReactionForce(const casadi::MX& euler_angle, const casadi::MX& omega) {
        casadi::MX N = casadi::MX::zeros(3, 1);
        N(0) = -(225008*sin(euler_angle(1)))/409 - (20160*omega(0)*omega(2))/409;
        N(1) =  (225008*cos(euler_angle(1))*sin(euler_angle(2)))/409 - (20160*omega(1)*omega(2))/409;
        N(2) =   686*cos(euler_angle(2))*cos(euler_angle(1));
        return N;
    }

    // Extract the yaw-only desired rotation matrix from the current human orientation.
    void getDesiredRotation();

    // Publish RViz sphere markers for left/right tactile targets and a cube for the desired pose.
    void publishTactileMarker();

    /**
     * @brief Rotation tracking cost term.
     *
     * Approximates ||log(R * R_d^T)||^2 using the ZYX Euler angle representation.
     * The argument is clamped to [-1,1] before acos to avoid NaN at singularities.
     * @param z  Yaw angle (rad).
     * @param y  Pitch angle (rad).
     * @param x  Roll angle (rad).
     */
    casadi::MX getRotationCost(const casadi::MX& z, const casadi::MX& y, const casadi::MX& x);

public:
    ros::Subscriber userpose_sub;
    ros::Subscriber tactile_left_sub;
    ros::Subscriber tactile_right_sub;
    ros::Publisher  traj_pub;
    ros::NodeHandle nh;
    ros::Rate       loop_rate;
    double          loop_time;
    util_msgs::trajectory trajectory_msg;

private:
    // ── Human state ──────────────────────────────────────────────────────────
    Vector3d contact_position_left;   // left contact in body frame
    Vector3d contact_position_right;  // right contact in body frame
    Vector3d dummy_tactile_left;      // desired left contact in body frame
    Vector3d dummy_tactile_right;     // desired right contact in body frame
    Vector3d dummy_tactile_left_world;
    Vector3d dummy_tactile_right_world;
    Vector3d euler_angle;             // ZYX Euler angles of human trunk
    Matrix3d human_rotation_matrix;   // R_world_body
    Vector3d human_position;          // CoM position in world frame
    Vector3d angular_vel;             // angular velocity in body frame
    Vector3d com_vel;                 // CoM velocity in body frame
    Vector3d foot_position;
    casadi::DM q0;                    // current state vector (18×1) passed to NMPC

    // ── NMPC parameters ──────────────────────────────────────────────────────
    static constexpr double p_h = 10;  // prediction horizon (steps)

    // ── Centroidal model constants ────────────────────────────────────────────
    static constexpr double m   = 70;    // human mass [kg]
    static constexpr double g   = -9.8;  // gravity [m/s²]
    static constexpr double c   = 1.8;   // human height [m]
    // Principal moments of inertia (thin rod + point mass approximation)
    static constexpr double Ixx = (0.2*0.2 + 1.8*1.8) * m;
    static constexpr double Iyy = (0.2*0.2 + 1.8*1.8) * m;
    static constexpr double Izz = (0.2*0.2 + 0.2*0.2) * m;
    Matrix3d I;
    casadi::MX inertial_matrix;      // I  (3×3, constant symbolic)
    casadi::MX inertial_matrix_inv;  // I⁻¹ (3×3, constant symbolic)
    Vector3d   L;                    // foot-to-CoM offset in body frame
    casadi::MX L_foot_com;           // CasADi version of L

    // ── Cost weights ─────────────────────────────────────────────────────────
    // Q: state error weights — [euler(0), pos(400), omega(10), vel(4), p_l(200), p_r(200)]
    // R: input weights       — [f1,f2(0.5 each), dp1,dp2(0.1 each)]
    casadi::DM Q_casadi;   // 18×18 diagonal
    casadi::DM R_casadi;   // 12×12 diagonal
    casadi::DM X_ref;      // reference state (18×1)

    // ── Unused / legacy ───────────────────────────────────────────────────────
    Matrix3d   rotation_matrix_ref;
    Vector3d   human_position_ref;
    Vector3d   velocity;
    MatrixXd   Q_weight, R_weight;

    // ── Solver outputs ────────────────────────────────────────────────────────
    Vector3d force_left,  force_right;   // optimal contact forces [N]
    Vector3d dpos_left,   dpos_right;    // optimal contact velocities [m/s]
    Vector3d contact_left, contact_right;

    // ── Misc ──────────────────────────────────────────────────────────────────
    bool   start_flag;
    double sum_time, count;  // for average loop-time logging

    // ── Performance logging ───────────────────────────────────────────────────
    bool     pose_changed_;          // true when a significant pose change was detected
    double   mpc_solve_time_ms_;     // last centroidalNMPC() wall time [ms]
    Vector3d last_logged_position_;  // human_position at last log event
    Matrix3d last_logged_rotation_;  // human_rotation_matrix at last log event
    std::string perf_log_path_;      // path to the output markdown file

    // ── RViz visualization ────────────────────────────────────────────────────
    visualization_msgs::Marker tactile_marker_left, tactile_marker_right, desired_marker;
    ros::Publisher marker_pub;
    geometry_msgs::TransformStamped transform_stamped;
    tf2_ros::TransformBroadcaster    tf_broadcaster;
    Matrix3d rotation_matrix_d;  // desired yaw-only rotation
    Vector3d euler_angle_d;      // desired ZYX Euler angles

    // ── Pre-built NMPC problem (constructed once in init) ─────────────────────
    casadi::Opti nmpc_;          // CasADi Opti stack; solver compiled at init time
    casadi::MX   X_var_;         // state trajectory variable  (18 × p_h)
    casadi::MX   F_var_;         // input trajectory variable  (12 × p_h)
    casadi::MX   q0_param_;      // parameter: initial state   (18 × 1)
    casadi::MX   X_ref_param_;   // parameter: reference state (18 × 1)
};

#endif
