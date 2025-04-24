/**
 * @file Trajectory.cpp
 * @brief Generate optimal trajectory 
 * @author Yu Sun,Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 04.19 2023
 */
#include <optimal_traj/Trajectory.hpp>

void OptimalTrajectory::init()
{
    // The distance between user and robot && The interact force to user && obstacle 
    userpose_sub = nh.subscribe("/dummy_human_pose",10, &OptimalTrajectory::poseCallback, this);
    tactile_left_sub = nh.subscribe("/contact_position_left",10, &OptimalTrajectory::tactileLeftCallback, this);
    tactile_right_sub = nh.subscribe("/contact_position_right",10, &OptimalTrajectory::tactileRightCallback, this);
    traj_pub = nh.advertise<util_msgs::trajectory>("/trajectory",10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("tactile_marker", 10);


    human_position << 1,0,0;
    loop_time = 1.0 / PLANNING_RATE;
    start_flag = true;

    inertial_matrix(0, 0) = Ixx;
    inertial_matrix(0, 1) = 0;
    inertial_matrix(0, 2) = 0;
    inertial_matrix(1, 0) = 0;
    inertial_matrix(1, 1) = Iyy;
    inertial_matrix(1, 2) = 0;
    inertial_matrix(2, 0) = 0;
    inertial_matrix(2, 1) = 0;
    inertial_matrix(2, 2) = Izz;
    inertial_matrix_inv(0, 0) = 1/Ixx;
    inertial_matrix_inv(0, 1) = 0;
    inertial_matrix_inv(0, 2) = 0;
    inertial_matrix_inv(1, 0) = 0;
    inertial_matrix_inv(1, 1) = 1/Iyy;
    inertial_matrix_inv(1, 2) = 0;
    inertial_matrix_inv(2, 0) = 0;
    inertial_matrix_inv(2, 1) = 0;
    inertial_matrix_inv(2, 2) = 1/Izz;

    for(int i = 0; i < 6; i++){
        R_casadi(i, i) = 0.5;
        R_casadi(i+6, i+6) = 0.1;
    }

    for(int i = 0; i < 3; i++){
        Q_casadi(i, i) = 0;
        Q_casadi(i+3, i+3) = 400;
        Q_casadi(i+6, i+6) = 10;
        Q_casadi(i+9, i+9) = 4;
        Q_casadi(i+12, i+12) = 200;
        Q_casadi(i+15, i+15) = 200;
    }
    // Q_casadi(0, 0) = 0;

    double desired_width = 0.33;

    X_ref(5) = c/2;
    X_ref(13) = desired_width;
    X_ref(16) = -desired_width;
    L << 0, 0, -c/2;

    human_rotation_matrix.setIdentity();

    contact_left << 1.15, desired_width, c/2;
    contact_right << 1.15, -desired_width, c/2;
    human_position << 1.15, 0, c/2; 

    dummy_tactile_left << 0, desired_width, 0;
    dummy_tactile_right << 0, -desired_width, 0;
}

void OptimalTrajectory::tactileLeftCallback(const geometry_msgs::Vector3ConstPtr &input)
{
    contact_left << input->x, input->y, input->z;
    contact_position_left = human_rotation_matrix.transpose()*(contact_left - human_position);
}

void OptimalTrajectory::tactileRightCallback(const geometry_msgs::Vector3ConstPtr &input)
{
    contact_right << input->x, input->y, input->z;
    contact_position_right = human_rotation_matrix.transpose()*(contact_right - human_position);
}

Eigen::Matrix3d eulerZYXToRotationMatrix(double yaw, double pitch, double roll) {
    // Create individual rotation matrices for each axis
    Eigen::Matrix3d R_z; // Rotation around Z-axis (yaw)
    Eigen::Matrix3d R_y; // Rotation around Y-axis (pitch)
    Eigen::Matrix3d R_x; // Rotation around X-axis (roll)

    // Z-axis rotation (yaw)
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
           0,         0,        1;

    // Y-axis rotation (pitch)
    R_y << cos(pitch),  0, sin(pitch),
           0,           1, 0,
           -sin(pitch), 0, cos(pitch);

    // X-axis rotation (roll)
    R_x << 1, 0,          0,
           0, cos(roll), -sin(roll),
           0, sin(roll),  cos(roll);

    // Combined rotation matrix (ZYX order: R = R_z * R_y * R_x)
    Eigen::Matrix3d R = R_z * R_y * R_x;

    return R;
}

void OptimalTrajectory::getDesiredRotation()
{

    double ag_d =  acos( human_rotation_matrix(0, 0) / sqrt(human_rotation_matrix(0, 0)*human_rotation_matrix(0, 0) + human_rotation_matrix(0, 1)*human_rotation_matrix(0, 1)));
    double aa_d = human_rotation_matrix(0,0);
    double ab_d = human_rotation_matrix(0,1);
    double ac_d = human_rotation_matrix(0,2);
    double ba_d = human_rotation_matrix(1,0);
    double bb_d = human_rotation_matrix(1,1);
    double bc_d = human_rotation_matrix(1,2);
    double ca_d = human_rotation_matrix(2,0);
    double cb_d = human_rotation_matrix(2,1);
    double cc_d = human_rotation_matrix(2,2);
    double d_d = sqrt(aa_d * aa_d + ba_d * ba_d);

    if (ac_d == 0 && bc_d == 0 && cc_d == 1) {
        rotation_matrix_d = human_rotation_matrix;
        return;
    }

    double phi_d = atan2(-ca_d, d_d);
    double theta_d = atan2(ba_d/cos(phi_d), aa_d/cos(phi_d));
    double beta_d = atan2(cb_d/cos(phi_d), cc_d/cos(phi_d));
   
    // theta = euler_angle(0);

    Matrix3d Rz_d;
    Rz_d(0, 0) = cos(theta_d); Rz_d(0, 1) = -sin(theta_d);
    Rz_d(1, 0) = sin(theta_d); Rz_d(1, 1) = cos(theta_d);
    Rz_d(2, 2) = 1;
    rotation_matrix_d.setZero();
    rotation_matrix_d = Rz_d;
    euler_angle_d(0) = theta_d;
    euler_angle_d(1) = 0;
    euler_angle_d(2) = 0;
    std::cout << "y: " << phi_d << std::endl;
    std::cout << "z: " << theta_d << std::endl;
    std::cout << "x: " << beta_d << std::endl;

    euler_angle_d << ag_d, 0, 0;
}

void OptimalTrajectory::poseCallback(const geometry_msgs::PoseStampedConstPtr &input)
{ 
    human_position << input->pose.position.x, input->pose.position.y, input->pose.position.z;  
    Eigen::Quaterniond quaternion(input->pose.orientation.w, input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z);
    human_rotation_matrix = quaternion.toRotationMatrix();
    //euler_angle = human_rotation_matrix.eulerAngles(2, 1, 0);
    double aa = human_rotation_matrix(0,0);
    double ab = human_rotation_matrix(0,1);
    double ac = human_rotation_matrix(0,2);
    double ba = human_rotation_matrix(1,0);
    double bb = human_rotation_matrix(1,1);
    double bc = human_rotation_matrix(1,2);
    double ca = human_rotation_matrix(2,0);
    double cb = human_rotation_matrix(2,1);
    double cc = human_rotation_matrix(2,2);
    double d = sqrt(aa*aa + ba*ba);

    if (ac== 0 && bc == 0 && c == 1) {
        rotation_matrix_d = human_rotation_matrix;
        return;
    }

    double phi = atan2(-ca, d);
    double theta = atan2(ba/cos(phi), aa/cos(phi));
    double beta = atan2(cb/cos(phi), cc/cos(phi));
    euler_angle(0) = theta;
    euler_angle(1) = phi;
    euler_angle(2) = beta;
}

casadi::MX safe_acos(const casadi::MX& param){
    casadi::MX clipped = fmin(fmax(param, -1), 1);
    return acos(clipped);
}

casadi::MX OptimalTrajectory::getRotationCost(const casadi::MX& z, const casadi::MX& y, const casadi::MX& x)
{
    // return acos(2 * cos(0.5 * x) * cos(0.5 * x) * cos(0.5 * y) * cos(0.5 * y) - 1) * acos(2 * cos(0.5 * x) * cos(0.5 * x) * cos(0.5 * y) * cos(0.5 * y) - 1);
    casadi::MX param = 2 * pow(cos(0.5*x),2) * pow(cos(0.5*y),2) - 1;
    param = casadi::MX::if_else(param >= 1, 
                            1, 
                            casadi::MX::if_else(param <= -1, 
                                            -1, 
                                            param));
    return 100000 * pow(acos(param), 2);
}

void OptimalTrajectory::getHumanPose()
{
    // human pose, Kalman filter etc.
    double cur_angle = 3.1415926/10;
    angular_vel << 0, 0, 0;
    com_vel << 0, 0, 0;


    contact_position_left = human_rotation_matrix.transpose()*(contact_left - human_position);
    contact_position_right = human_rotation_matrix.transpose()*(contact_right - human_position);

    // q0
    getDesiredRotation();
    std::cout << "human_rotation_matrix: " << human_rotation_matrix << std::endl;
    q0(0) = euler_angle(0);
    q0(1) = euler_angle(1);
    q0(2) = euler_angle(2);
    // std::cout << "desire rotation matrix: " << eulerZYXToRotationMatrix(euler_angle(0), 0, 0) << std::endl;
    q0(3) = human_position(0);
    q0(4) = human_position(1);
    q0(5) = human_position(2);
    q0(6) = angular_vel(0);
    q0(7) = angular_vel(1);
    q0(8) = angular_vel(2);
    q0(9) = com_vel(0);
    q0(10) = com_vel(1);
    q0(11) = com_vel(2);
    q0(12) = dummy_tactile_left(0);
    q0(13) = dummy_tactile_left(1);
    q0(14) = dummy_tactile_left(2);
    q0(15) = dummy_tactile_right(0);
    q0(16) = dummy_tactile_right(1);
    q0(17) = dummy_tactile_right(2);
    
    Vector3d pos_base;
    pos_base = human_position + human_rotation_matrix*L;
    // std::cout << "pos_base: " << pos_base.transpose() << std::endl;

    // X_ref(0) = q0(0);
    X_ref(0) = euler_angle_d(0);
    X_ref(1) = euler_angle_d(1);
    X_ref(2) = euler_angle_d(2);

    X_ref(3) = pos_base(0);
    X_ref(4) = pos_base(1);
    X_ref(5) = pos_base(2) + c/2;

    std::cout << "q0: " << q0 << std::endl;
    std::cout << "X_ref: " << X_ref << std::endl;
}

void OptimalTrajectory::centroidalNMPC()
{
    using namespace casadi;

    // casadi initialization 
    Opti nmpc;
    MX X = nmpc.variable(18,p_h);
    MX F = nmpc.variable(12,p_h);
    nmpc.subject_to(X(Slice(), 0) == q0);

    for (int k = 0; k < p_h-1; ++k) {
        // Dynamics constraint
        double delta_t = 0.2;
        MX dX = centroidalDynamics(X(Slice(), k), F(Slice(), k));
        nmpc.subject_to( X(Slice(), k+1) == X(Slice(), k) + delta_t*dX );

        // Friction cone constraints
        nmpc.subject_to(F(0, k) == 0);
        nmpc.subject_to(F(1, k) <= 0);
        nmpc.subject_to(F(2, k) == 0);
        nmpc.subject_to(F(3, k) == 0);
        nmpc.subject_to(F(4, k) >= 0);
        nmpc.subject_to(F(5, k) == 0);
        nmpc.subject_to(F(6, k) == 0);
        nmpc.subject_to(F(7, k) == 0);
        nmpc.subject_to(F(9, k) == 0);
        nmpc.subject_to(F(10, k) == 0);

        // // Lower bound and upper bound
        nmpc.subject_to(-200 <= F(1, k));
        nmpc.subject_to(F(4, k) <= 200);
        nmpc.subject_to(-SLIP_MAX_SPEED <= F(8, k));
        nmpc.subject_to(F(8, k) <= SLIP_MAX_SPEED);
        nmpc.subject_to(F(11, k) >= -SLIP_MAX_SPEED);
        nmpc.subject_to(F(11, k) <= SLIP_MAX_SPEED);

        nmpc.subject_to(q0(14) + F(8, k)*loop_time >= -0.2);
        nmpc.subject_to(q0(14) + F(8, k)*loop_time <= 0.35);
        nmpc.subject_to(q0(17) + F(11, k)*loop_time <= 0.35);
        nmpc.subject_to(q0(17) + F(11, k)*loop_time >= -0.2);
    }
    MX J = MX::zeros(1, 1);
    for (int k = 0; k < p_h; ++k) {
        J = J + 
            mtimes(
                mtimes((X_ref.T() - X(Slice(), k).T()), Q_casadi), 
                (X_ref - X(Slice(), k))
            ) 
            + getRotationCost(X(0, k), X(1, k), X(2, k))
            + mtimes(
                mtimes((F(Slice(), k).T()), R_casadi), 
                (F(Slice(), k))
            );
    }
    nmpc.minimize(J);
    casadi::Dict casadiOptions;
    casadiOptions["ipopt.linear_solver"] = "mumps";  
    casadiOptions["ipopt.max_iter"] = 1000;  
    casadiOptions["ipopt.print_level"] = 1;
    casadiOptions["ipopt.warm_start_init_point"] = "yes";
    casadiOptions["ipopt.acceptable_tol"]        = 1e-5;
    casadiOptions["ipopt.tol"]        = 1e-5;
    nmpc.solver("ipopt", casadiOptions);
    OptiSol sol = nmpc.solve();   // actual solve
    // solution
    DM f_all = sol.value(F);
    DM f_first = f_all(Slice(), 0);
    std::cout << "solution: " << f_first << std::endl;
    force_left << static_cast<double>(f_first(0).scalar()), static_cast<double>(f_first(1).scalar()), static_cast<double>(f_first(2).scalar());
    force_right << static_cast<double>(f_first(3).scalar()), static_cast<double>(f_first(4).scalar()), static_cast<double>(f_first(5).scalar());
    dpos_left << static_cast<double>(f_first(6).scalar()), static_cast<double>(f_first(7).scalar()), static_cast<double>(f_first(8).scalar());
    dpos_right << static_cast<double>(f_first(9).scalar()), static_cast<double>(f_first(10).scalar()), static_cast<double>(f_first(11).scalar());
}

casadi::MX OptimalTrajectory::centroidalDynamics(const casadi::MX& current_state, const casadi::MX& input) {

    // using namespace casadi;
    // State
    casadi::MX theta = current_state(casadi::Slice(0, 3));
    casadi::MX p = current_state(casadi::Slice(3, 6));
    casadi::MX omega = current_state(casadi::Slice(6, 9));
    casadi::MX v = current_state(casadi::Slice(9, 12));
    casadi::MX p1 = current_state(casadi::Slice(12, 15));
    casadi::MX p2 = current_state(casadi::Slice(15, 18));
    
    // Input
    casadi::MX f1 = input(casadi::Slice(0, 3));
    casadi::MX f2 = input(casadi::Slice(3, 6));
    casadi::MX dp1 = input(casadi::Slice(6, 9));
    casadi::MX dp2 = input(casadi::Slice(9, 12));

    casadi::MX z = theta(0), y = theta(1), x = theta(2);
    // ZYX Euler angles to angular velocity (inverse)
    casadi::MX T = casadi::MX::zeros(3, 3);
    T(0, 1) = -sin(z);
    T(0, 2) = cos(z)*cos(y);
    T(1, 1) = cos(z);
    T(1, 2) = cos(y)*sin(z);
    T(2, 0) = 1;
    T(2, 2) = -sin(y);
    
    // // Rotation matrix
    casadi::MX Rz = casadi::MX::zeros(3, 3), Ry = casadi::MX::zeros(3, 3), Rx = casadi::MX::zeros(3, 3), R = casadi::MX::zeros(3, 3);
    Rz(0, 0) = cos(z); Rz(0, 1) = -sin(z);
    Rz(1, 0) = sin(z); Rz(1, 1) = cos(z);
    Rz(2, 2) = 1;
    Ry(0, 0) = cos(y); Ry(0, 2) = sin(y);
    Ry(1, 1) = 1;
    Ry(2, 0) = -sin(y); Ry(2, 2) = cos(y);
    Rx(0, 0) = 1;
    Rx(1, 1) = cos(x); Rx(1, 2) = -sin(x);
    Rx(2, 1) = sin(x); Rx(2, 2) = cos(x);
    R = Rz * Ry * Rx;

    casadi::MX dtheta = mtimes(T, omega);
    casadi::MX dp = mtimes(R, v);
    casadi::MX N = groundReactionForce(theta, omega);

    casadi::MX domega =  mtimes(inertial_matrix_inv , (-cross(omega, mtimes(inertial_matrix, omega)) + cross(p1, f1) + cross(p2, f2) + cross(L_foot_com, N)));
    // casadi::MX domega =  -cross(omega, mtimes(inertial_matrix, omega)) + cross(p1, f1) + cross(p2, f2) + cross(L_foot_com, N);
    casadi::MX dv = (f1 + f2 + N + mtimes(transpose(R), m * g * casadi::MX::vertcat({0, 0, 1}))) / m;
    
    casadi::MX dX = casadi::MX::zeros(18);
    dX(casadi::Slice(0, 3)) = dtheta;
    dX(casadi::Slice(3, 6)) = dp;
    dX(casadi::Slice(6, 9)) = domega;
    dX(casadi::Slice(9, 12)) = dv;
    dX(casadi::Slice(12, 15)) = dp1;
    dX(casadi::Slice(15, 18)) = dp2;
    return dX;
}

void OptimalTrajectory::publishCommand()
{
    trajectory_msg.force_left.x = force_left[0];
    trajectory_msg.force_left.y = force_left[1];
    trajectory_msg.force_left.z = force_left[2];
    trajectory_msg.force_right.x = force_right[0];
    trajectory_msg.force_right.y = force_right[1];
    trajectory_msg.force_right.z = force_right[2];
    trajectory_msg.dp_left.x = dummy_tactile_left_world[0];
    trajectory_msg.dp_left.y = dummy_tactile_left_world[1];
    trajectory_msg.dp_left.z = dummy_tactile_left_world[2];
    trajectory_msg.dp_right.x = dummy_tactile_right_world[0];
    trajectory_msg.dp_right.y = dummy_tactile_right_world[1];
    trajectory_msg.dp_right.z = dummy_tactile_right_world[2];
    dummy_tactile_left += dpos_left * loop_time;
    dummy_tactile_right += dpos_right * loop_time;
    // std::cout << "dummy_tactile_left: " << dummy_tactile_left.transpose() << std::endl;
    // std::cout << "dummy_tactile_right: " << dummy_tactile_right.transpose() << std::endl;
    traj_pub.publish(trajectory_msg);
}

void OptimalTrajectory::publishTactileMarker(){
    // frame transfer
    dummy_tactile_left_world = human_rotation_matrix * dummy_tactile_left + human_position;
    dummy_tactile_right_world = human_rotation_matrix * dummy_tactile_right + human_position;

    tactile_marker_left.id = 0;
    tactile_marker_left.type = visualization_msgs::Marker::SPHERE;
    tactile_marker_left.ns = "optimal_traj";

    tactile_marker_left.pose.orientation.x = 0.0;
    tactile_marker_left.pose.orientation.y = 0.0;
    tactile_marker_left.pose.orientation.z = 0.0;
    tactile_marker_left.pose.orientation.w = 0.0;

    tactile_marker_left.header.frame_id = "world";

    tactile_marker_left.pose.position.x = dummy_tactile_left_world[0];
    tactile_marker_left.pose.position.y = dummy_tactile_left_world[1];
    tactile_marker_left.pose.position.z = dummy_tactile_left_world[2];

    tactile_marker_left.scale.x = 0.2;
    tactile_marker_left.scale.y = 0.2;
    tactile_marker_left.scale.z = 0.2;

    tactile_marker_left.color.a = 0.8;
    tactile_marker_left.color.g = 0.4;
    tactile_marker_left.color.r = 0.4;

    tactile_marker_right.id = 1;
    tactile_marker_right.type = visualization_msgs::Marker::SPHERE;
    tactile_marker_right.ns = "optimal_traj";

    tactile_marker_right.pose.orientation.x = 0.0;
    tactile_marker_right.pose.orientation.y = 0.0;
    tactile_marker_right.pose.orientation.z = 0.0;
    tactile_marker_right.pose.orientation.w = 0.0;

    tactile_marker_right.header.frame_id = "world";

    tactile_marker_right.pose.position.x = dummy_tactile_right_world[0];
    tactile_marker_right.pose.position.y = dummy_tactile_right_world[1];
    tactile_marker_right.pose.position.z = dummy_tactile_right_world[2];

    tactile_marker_right.scale.x = 0.2;
    tactile_marker_right.scale.y = 0.2;
    tactile_marker_right.scale.z = 0.2;

    tactile_marker_right.color.a = 0.8;
    tactile_marker_right.color.g = 0.4;
    tactile_marker_right.color.r = 0.4;

    desired_marker.id = 2;
    desired_marker.type = visualization_msgs::Marker::CUBE;
    desired_marker.ns = "optimal_traj";

    Eigen::Quaterniond quaternion(rotation_matrix_d);
    std::cout << "rotation_matrix_d: " << rotation_matrix_d << std::endl;

    desired_marker.pose.orientation.x = quaternion.x();
    desired_marker.pose.orientation.y = quaternion.y();
    desired_marker.pose.orientation.z = quaternion.z();
    desired_marker.pose.orientation.w = quaternion.w();

    desired_marker.header.frame_id = "world";

    desired_marker.pose.position.x = human_position[0];
    desired_marker.pose.position.y = human_position[1];
    desired_marker.pose.position.z = human_position[2];

    desired_marker.scale.x = 0.2;
    desired_marker.scale.y = 0.4;
    desired_marker.scale.z = 1.8;

    desired_marker.color.a = 0.7;
    desired_marker.color.b = 1.0;
    // desired_marker.color.r = 0.4;

    transform_stamped.header.stamp = ros::Time::now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id = 3;
    transform_stamped.transform.translation.x = human_position[0];
    transform_stamped.transform.translation.y = human_position[1];
    transform_stamped.transform.translation.z = human_position[2];
    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.x();
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();
    tf_broadcaster.sendTransform(transform_stamped);
    

    marker_pub.publish(tactile_marker_right);
    marker_pub.publish(tactile_marker_left);
    marker_pub.publish(desired_marker);
}

void OptimalTrajectory::spin()
{
    while (start_flag == false && ros::ok())
    {
        ROS_WARN_STREAM("Waiting for intent detection");
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO_STREAM("Intent detected, program starts");
    while (ros::ok())
    {
        ros::spinOnce();
        double t1 = ros::Time::now().toSec();

        getHumanPose();
        centroidalNMPC();
        publishTactileMarker();
        publishCommand();
        // calculate average time
        double t2 = ros::Time::now().toSec()-t1;
        sum_time += 1000*t2;
        count += 1;
        std::cout << "average time : " << sum_time/count << " ms" << std::endl << std::endl;
        loop_rate.sleep();
    }
}