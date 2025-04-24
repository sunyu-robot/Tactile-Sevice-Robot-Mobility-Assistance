/**
 * @file Trajectory.hpp
 * @brief Generate optimal trajectory
 * @author Yu Sun,Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 04.19 2023
 */
#ifndef optimal_trajectory
#define optimal_trajectory

#include <float.h>
#include <ros/ros.h>
#include <math.h>
// #include "IpIpoptApplication.hpp"
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <util_msgs/trajectory.h>
#include <vector>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64.h>
#include <util_msgs/trajectory.h>
#include <casadi/casadi.hpp>
#include <cmath>
// casadi
#include "visualization_msgs/Marker.h"
//tf
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#define PLANNING_RATE 10.0
#define SLIP_MAX_SPEED 2.0

using namespace Eigen;
// using namespace casadi;

typedef Matrix<double, 15, 1> Vector15d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;

class OptimalTrajectory
{
public:
    OptimalTrajectory() : loop_rate(PLANNING_RATE), inertial_matrix(3, 3), inertial_matrix_inv(3, 3), L_foot_com(3, 1), Q_casadi(18, 18), R_casadi(12, 12), X_ref(18, 1), q0(18, 1)
    {
        init();
    }

    /**
    * @brief Init params 
    * @detail
    *  Get params from config files and init params
    */
    void init();

    /**
    * @brief Control loop
    * @detail
    *  Control loop
    */
    void spin();

    /**
    * @brief User pose callback
    * @detail
    *  User pose callback
    */
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &input);

    /**
    * @brief Tactile contact callback (left)
    * @detail
    *  Tactile contact callback (left)
    */
    void tactileLeftCallback(const geometry_msgs::Vector3ConstPtr &input);

    /**
    * @brief Tactile contact callback (right)
    * @detail
    *  Tactile contact callback (right)
    */
    void tactileRightCallback(const geometry_msgs::Vector3ConstPtr &input);

    /**
    * @brief Publish command
    * @detail
    *   Publish command force, position
    */    
    void publishCommand();

    /**
    * @brief get the Human pose
    * @detail
    *  get the Human pose after coordinate transformation and kalman filter
    */
    void getHumanPose(); 

    /**
    * @brief constrainAngle function
    * @detail
    *  constraint the angle to -pi - pi
    * @param[in] &input      a double type angle
    * @param[out] &output    angle which is limited to -pi to pi   
    */
    inline double constrainAngle(double angle)
    {
        angle = fmod(angle, 2 * M_PI); 
        
        // 如果角度大于 pi，减去 2*pi
        if (angle > M_PI)
            angle -= 2 * M_PI;
        // 如果角度小于 -pi，加上 2*pi
        else if (angle < -M_PI)
            angle += 2 * M_PI;

        return angle;
    }

    /**
    * @brief Low pass fitler function
    * @detail
    *  constraint the angle to 0 - 2pi
    * @param[in] &input      a double type angle
    * @param[out] &output    angle which is limited to 0 to 2pi   
    */
    inline double lowpassFilter(double last, double current, double alpha = 0.9)
    {
        double output;
        output = alpha * last + (1.0 - alpha) * current;
        return output;
    }

    /**
    * @brief calculate control input
    * @detail
    * @param[in] : current state desired state(cost function, rotation matrix, position)
    * @param[out] : f1 f2 dp1 dp2 (defined in body frame)
    */
    void centroidalNMPC();     

    /**
    * @brief calculate centroidal model dynamics
    * @detail
    *  control input : f1 f2 dp1 dp2 (defined in body frame)
    *  state : phi(euler angle zyx) p angel velocity (body frame) velocity (body frame) angel acc vel acc contact position (left and right)
    */
    casadi::MX centroidalDynamics(const casadi::MX& current_state, const casadi::MX& input);          

    /**
    * @brief calculate ground reaction force
    * @detail
    *  @param[in] : Rotation matirx
    *  @param[out] : Reaction force (body frame)
    */
    inline casadi::MX groundReactionForce(const casadi::MX& euler_angle, const casadi::MX& omega) {
        casadi::MX N = casadi::MX::zeros(3, 1);
        // N(0) = -(4704 * sin(euler_angle(1))) / 605 * (cos(euler_angle(1)) * cos(euler_angle(1)) + sin(euler_angle(1)) * sin(euler_angle(1)));
        // N(1) = (4704 * cos(euler_angle(1)) * sin(euler_angle(2))) / (605 * (cos(euler_angle(2)) * cos(euler_angle(2)) * cos(euler_angle(1)) * cos(euler_angle(1)) + cos(euler_angle(2)) * cos(euler_angle(2)) * sin(euler_angle(1)) * sin(euler_angle(1)) + cos(euler_angle(1)) * cos(euler_angle(1)) * sin(euler_angle(2)) * sin(euler_angle(2)) + sin(euler_angle(2)) * sin(euler_angle(2)) * sin(euler_angle(1)) * sin(euler_angle(1))));
        // N(2) = (49 * cos(euler_angle(2)) * cos(euler_angle(1))) /(5 * (cos(euler_angle(2)) * cos(euler_angle(2)) * cos(euler_angle(1)) * cos(euler_angle(1)) + cos(euler_angle(2)) * cos(euler_angle(2)) * sin(euler_angle(1)) * sin(euler_angle(1)) + cos(euler_angle(1)) * cos(euler_angle(1)) * sin(euler_angle(2)) * sin(euler_angle(2)) + sin(euler_angle(2)) * sin(euler_angle(2)) * sin(euler_angle(1)) * sin(euler_angle(1))));
        N(0) = - (225008*sin(euler_angle(1)))/409 - (20160*omega(0)*omega(2))/409;
        N(1) = (225008*cos(euler_angle(1))*sin(euler_angle(2)))/409 - (20160*omega(1)*omega(2))/409;
        N(2) = 686*cos(euler_angle(2))*cos(euler_angle(1));
        return N;
    }        

    // get the desired rotation matrix
    void getDesiredRotation();

    // publish the desire tactile contact position
    void publishTactileMarker();

    // get the rotation cost -- log(R * R_d')
    casadi::MX getRotationCost(const casadi::MX& z, const casadi::MX& y, const casadi::MX& x);
public:
    // userpose Subscriber
    ros::Subscriber userpose_sub;
    // tactile sub
    ros::Subscriber tactile_left_sub;
    // tactile sub
    ros::Subscriber tactile_right_sub;
    // trajectory Publisher
    ros::Publisher traj_pub;
    // Node handle
    ros::NodeHandle nh;
    // Loop rate
    ros::Rate loop_rate;    
    // loop time
    double loop_time;
    // trajectory
    util_msgs::trajectory trajectory_msg;
private: 
    // state (from state estimator and robot interface)
    // current human state
    Vector3d contact_position_left;
    // current human state
    Vector3d contact_position_right;
    // dummy tactile
    Vector3d dummy_tactile_left, dummy_tactile_right;
    // dummy tactile world
    Vector3d dummy_tactile_left_world, dummy_tactile_right_world;
    // current human pose 
    // Euler angle of user (ZYX Euler angle) 
    Vector3d euler_angle;
    // huamn rotation matrix
    Matrix3d human_rotation_matrix;

    // human COM position (world frame)
    Vector3d human_position;
    // human angular velocity (omega I B)
    Vector3d angular_vel;
    // human COM velocity (body frame)
    Vector3d com_vel;
    // human foot position
    Vector3d foot_position;
    // current state
    casadi::DM q0;

    // cassadi based optimazation
    // variable
    // casadi::MX X_, F_, X0_;
    // preditive horizion
    static constexpr double p_h = 5;
    // centroidal model
    // human mass
    static constexpr double m = 70;
    // gravity
    static constexpr double g = -9.8;
    // Inertial matrix
    static constexpr double Ixx = (0.2*0.2 + 1.8*1.8) * m;  // adjust this formula according to your actual needs
    static constexpr double Iyy = (0.2*0.2 + 1.8*1.8) * m;  // assuming symmetry in euler_angle(1)
    static constexpr double Izz = (0.2*0.2 + 0.2*0.2) * m;  // assuming symmetry in z
    Matrix3d I;
    // casadi representation
    casadi::MX inertial_matrix;
    casadi::MX inertial_matrix_inv;
    // weight in casadi representation
    casadi::DM Q_casadi, R_casadi;
    // refrence 
    casadi::DM X_ref;
    
    // foot - com
    static constexpr double c = 1.8;
    // length from the foot to the com
    Vector3d L;
    // casadi
    casadi::MX L_foot_com;
    // reference traj
    Matrix3d rotation_matrix_ref;
    // reference position
    Vector3d human_position_ref;
    // user current state
    Vector3d velocity;
    // weight matrix state-input
    MatrixXd Q_weight;
    // weight matrix input
    MatrixXd R_weight;
    // solution 
    Vector3d force_left,  force_right;
    Vector3d dpos_left,  dpos_right;

    Vector3d contact_left, contact_right;

    // for the entire procedure
    bool start_flag;
    // sum time once a looprate
    double sum_time, count;

    // tactile marker
    visualization_msgs::Marker tactile_marker_left, tactile_marker_right, desired_marker;
    // marker publisher
    ros::Publisher marker_pub;
    // desired TF
    geometry_msgs::TransformStamped transform_stamped;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    // desired rotation matrix
    Matrix3d rotation_matrix_d;
    // desired euler angle
    Vector3d euler_angle_d;
    // desird quaternion

};

#endif