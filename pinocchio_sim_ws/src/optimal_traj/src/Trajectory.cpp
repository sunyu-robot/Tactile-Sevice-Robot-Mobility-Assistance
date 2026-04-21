/**
 * @file Trajectory.cpp
 * @brief Centroidal NMPC planner for human mobility assistance
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 04.19 2023
 */
#include <optimal_traj/Trajectory.hpp>
#include <fstream>
#include <iomanip>

void OptimalTrajectory::init()
{
    userpose_sub      = nh.subscribe("/dummy_human_pose",       10, &OptimalTrajectory::poseCallback,         this);
    tactile_left_sub  = nh.subscribe("/contact_position_left",  10, &OptimalTrajectory::tactileLeftCallback,  this);
    tactile_right_sub = nh.subscribe("/contact_position_right", 10, &OptimalTrajectory::tactileRightCallback, this);
    traj_pub   = nh.advertise<util_msgs::trajectory>("/trajectory", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("tactile_marker", 10);

    human_position << 1, 0, 0;
    loop_time  = 1.0 / PLANNING_RATE;
    start_flag = true;

    // Diagonal inertia matrix and its inverse (principal axes, thin-rod model)
    inertial_matrix(0,0) = Ixx; inertial_matrix(1,1) = Iyy; inertial_matrix(2,2) = Izz;
    inertial_matrix_inv(0,0) = 1.0/Ixx; inertial_matrix_inv(1,1) = 1.0/Iyy; inertial_matrix_inv(2,2) = 1.0/Izz;

    // Input cost R: force channels (0-5) weighted 0.5, velocity channels (6-11) weighted 0.1
    for (int i = 0; i < 6; i++) {
        R_casadi(i,   i)   = 0.5;
        R_casadi(i+6, i+6) = 0.1;
    }

    // State cost Q per axis group: euler(0), pos(400), omega(10), vel(4), p_l(200), p_r(200)
    for (int i = 0; i < 3; i++) {
        Q_casadi(i,    i)    = 0;
        Q_casadi(i+3,  i+3)  = 400;
        Q_casadi(i+6,  i+6)  = 10;
        Q_casadi(i+9,  i+9)  = 4;
        Q_casadi(i+12, i+12) = 200;
        Q_casadi(i+15, i+15) = 200;
    }

    double desired_width = 0.33;  // lateral half-width between contacts [m]

    X_ref(5)  = c / 2;
    X_ref(13) =  desired_width;
    X_ref(16) = -desired_width;
    L << 0, 0, -c / 2;

    human_rotation_matrix.setIdentity();
    last_logged_position_.setZero();
    last_logged_rotation_.setIdentity();
    pose_changed_    = false;
    mpc_solve_time_ms_ = 0.0;
    perf_log_path_   = "/workspace/pinocchio_sim_ws/src/perf_log.md";

    // Write markdown header (truncate on each fresh start)
    std::ofstream f(perf_log_path_, std::ios::trunc);
    f << "# Performance Log\n\n"
      << "| time (s) | event | mpc_solve (ms) | qp_solve (ms) | tracking_lag (ms) |\n"
      << "|----------|-------|---------------|--------------|------------------|\n";
    f.close();

    contact_left   << 1.15,  desired_width, c / 2;
    contact_right  << 1.15, -desired_width, c / 2;
    human_position << 1.15, 0,              c / 2;

    dummy_tactile_left  << 0,  desired_width, 0;
    dummy_tactile_right << 0, -desired_width, 0;

    // ── Build NMPC problem once ───────────────────────────────────────────────
    // The symbolic graph, constraints, and cost are compiled here.
    // Each call to centroidalNMPC() only updates q0_param_ / X_ref_param_
    // and re-runs the IPOPT solve — no graph reconstruction overhead.
    using namespace casadi;
    X_var_       = nmpc_.variable(18, p_h);
    F_var_       = nmpc_.variable(12, p_h);
    q0_param_    = nmpc_.parameter(18, 1);
    X_ref_param_ = nmpc_.parameter(18, 1);

    // Initial condition
    nmpc_.subject_to(X_var_(Slice(), 0) == q0_param_);

    for (int k = 0; k < p_h - 1; ++k) {
        // Euler integration of centroidal dynamics
        const double delta_t = 0.1;
        MX dX = centroidalDynamics(X_var_(Slice(), k), F_var_(Slice(), k));
        nmpc_.subject_to(X_var_(Slice(), k+1) == X_var_(Slice(), k) + delta_t * dX);

        // Friction cone: only normal (y) and tangential (z) forces are free per contact
        nmpc_.subject_to(F_var_(0,  k) == 0);
        nmpc_.subject_to(F_var_(1,  k) <= 0);
        nmpc_.subject_to(F_var_(2,  k) == 0);
        nmpc_.subject_to(F_var_(3,  k) == 0);
        nmpc_.subject_to(F_var_(4,  k) >= 0);
        nmpc_.subject_to(F_var_(5,  k) == 0);
        nmpc_.subject_to(F_var_(6,  k) == 0);
        nmpc_.subject_to(F_var_(7,  k) == 0);
        nmpc_.subject_to(F_var_(9,  k) == 0);
        nmpc_.subject_to(F_var_(10, k) == 0);

        // Force magnitude bounds
        nmpc_.subject_to(-200 <= F_var_(1, k));
        nmpc_.subject_to(F_var_(4, k) <= 200);

        // Contact sliding speed bounds
        nmpc_.subject_to(-SLIP_MAX_SPEED <= F_var_(8,  k));
        nmpc_.subject_to(F_var_(8,  k) <= SLIP_MAX_SPEED);
        nmpc_.subject_to(F_var_(11, k) >= -SLIP_MAX_SPEED);
        nmpc_.subject_to(F_var_(11, k) <= SLIP_MAX_SPEED);

        // Contact position reachability (body z-axis, relative to current q0)
        nmpc_.subject_to(q0_param_(14) + F_var_(8,  k) * loop_time >= -0.2);
        nmpc_.subject_to(q0_param_(14) + F_var_(8,  k) * loop_time <= 0.35);
        nmpc_.subject_to(q0_param_(17) + F_var_(11, k) * loop_time <= 0.35);
        nmpc_.subject_to(q0_param_(17) + F_var_(11, k) * loop_time >= -0.2);
    }

    // Cost: state tracking + rotation geodesic + input regularisation
    MX J = MX::zeros(1, 1);
    for (int k = 0; k < p_h; ++k) {
        MX e = X_ref_param_ - X_var_(Slice(), k);
        J = J + mtimes(mtimes(e.T(), Q_casadi), e)
              + getRotationCost(X_var_(0, k), X_var_(1, k), X_var_(2, k))
              + mtimes(mtimes(F_var_(Slice(), k).T(), R_casadi), F_var_(Slice(), k));
    }
    nmpc_.minimize(J);

    casadi::Dict opts;
    opts["ipopt.linear_solver"]         = "ma27";
    opts["ipopt.max_iter"]              = 100;
    opts["ipopt.print_level"]           = 1;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.mu_strategy"]           = "adaptive";
    opts["ipopt.acceptable_tol"]        = 1e-5;
    opts["ipopt.tol"]                   = 1e-5;
    nmpc_.solver("ipopt", opts);
}

void OptimalTrajectory::tactileLeftCallback(const geometry_msgs::Vector3ConstPtr &input)
{
    contact_left << input->x, input->y, input->z;
    contact_position_left = human_rotation_matrix.transpose() * (contact_left - human_position);
}

void OptimalTrajectory::tactileRightCallback(const geometry_msgs::Vector3ConstPtr &input)
{
    contact_right << input->x, input->y, input->z;
    contact_position_right = human_rotation_matrix.transpose() * (contact_right - human_position);
}

Eigen::Matrix3d eulerZYXToRotationMatrix(double yaw, double pitch, double roll) {
    Eigen::Matrix3d R_z, R_y, R_x;
    R_z << cos(yaw), -sin(yaw), 0,
           sin(yaw),  cos(yaw), 0,
           0,         0,        1;
    R_y << cos(pitch),  0, sin(pitch),
           0,           1, 0,
           -sin(pitch), 0, cos(pitch);
    R_x << 1, 0,           0,
           0, cos(roll), -sin(roll),
           0, sin(roll),  cos(roll);
    return R_z * R_y * R_x;
}

void OptimalTrajectory::getDesiredRotation()
{
    double aa_d = human_rotation_matrix(0,0), ab_d = human_rotation_matrix(0,1);
    double ac_d = human_rotation_matrix(0,2);
    double ba_d = human_rotation_matrix(1,0), bc_d = human_rotation_matrix(1,2);
    double ca_d = human_rotation_matrix(2,0), cb_d = human_rotation_matrix(2,1);
    double cc_d = human_rotation_matrix(2,2);
    double d_d  = sqrt(aa_d*aa_d + ba_d*ba_d);

    double ag_d = acos(aa_d / sqrt(aa_d*aa_d + ab_d*ab_d));

    // Gimbal-lock guard: identity rotation when pointing straight up
    if (ac_d == 0 && bc_d == 0 && cc_d == 1) {
        rotation_matrix_d = human_rotation_matrix;
        return;
    }

    double phi_d   = atan2(-ca_d, d_d);
    double theta_d = atan2(ba_d / cos(phi_d), aa_d / cos(phi_d));
    double beta_d  = atan2(cb_d / cos(phi_d), cc_d / cos(phi_d));

    // Keep only yaw component for the desired rotation
    Matrix3d Rz_d;
    Rz_d.setZero();
    Rz_d(0,0) = cos(theta_d); Rz_d(0,1) = -sin(theta_d);
    Rz_d(1,0) = sin(theta_d); Rz_d(1,1) =  cos(theta_d);
    Rz_d(2,2) = 1;
    rotation_matrix_d = Rz_d;

    euler_angle_d << ag_d, 0, 0;

    std::cout << "desired euler (y/z/x): " << phi_d << " / " << theta_d << " / " << beta_d << std::endl;
}

void OptimalTrajectory::poseCallback(const geometry_msgs::PoseStampedConstPtr &input)
{
    human_position << input->pose.position.x, input->pose.position.y, input->pose.position.z;
    Eigen::Quaterniond q(input->pose.orientation.w,
                         input->pose.orientation.x,
                         input->pose.orientation.y,
                         input->pose.orientation.z);
    human_rotation_matrix = q.toRotationMatrix();

    // Detect significant rotation change (>0.02 rad) as fall event trigger
    double rot_delta = (human_rotation_matrix * last_logged_rotation_.transpose()
                        - Matrix3d::Identity()).norm();
    if (rot_delta > 0.02) {
        pose_changed_ = true;
        last_logged_rotation_ = human_rotation_matrix;
    }

    double aa = human_rotation_matrix(0,0), ba = human_rotation_matrix(1,0);
    double ac = human_rotation_matrix(0,2), bc = human_rotation_matrix(1,2);
    double ca = human_rotation_matrix(2,0), cb = human_rotation_matrix(2,1);
    double cc = human_rotation_matrix(2,2);
    double d  = sqrt(aa*aa + ba*ba);

    if (ac == 0 && bc == 0 && c == 1) {
        rotation_matrix_d = human_rotation_matrix;
        return;
    }

    double phi   = atan2(-ca, d);
    double theta = atan2(ba / cos(phi), aa / cos(phi));
    double beta  = atan2(cb / cos(phi), cc / cos(phi));
    euler_angle << theta, phi, beta;
}

casadi::MX safe_acos(const casadi::MX& param) {
    casadi::MX clipped = fmin(fmax(param, -1), 1);
    return acos(clipped);
}

casadi::MX OptimalTrajectory::getRotationCost(const casadi::MX& z, const casadi::MX& y, const casadi::MX& x)
{
    // Geodesic distance approximation: ||log(R)||² via half-angle formula
    casadi::MX param = 2 * pow(cos(0.5*x), 2) * pow(cos(0.5*y), 2) - 1;
    param = casadi::MX::if_else(param >= 1,  1,
            casadi::MX::if_else(param <= -1, -1, param));
    return 100000 * pow(acos(param), 2);
}

void OptimalTrajectory::getHumanPose()
{
    angular_vel.setZero();
    com_vel.setZero();

    contact_position_left  = human_rotation_matrix.transpose() * (contact_left  - human_position);
    contact_position_right = human_rotation_matrix.transpose() * (contact_right - human_position);

    getDesiredRotation();
    std::cout << "human_rotation_matrix:\n" << human_rotation_matrix << std::endl;

    // Pack current state into q0 (18×1): euler, pos, omega, vel, p_left, p_right
    q0(0)  = euler_angle(0); q0(1)  = euler_angle(1); q0(2)  = euler_angle(2);
    q0(3)  = human_position(0); q0(4)  = human_position(1); q0(5)  = human_position(2);
    q0(6)  = angular_vel(0); q0(7)  = angular_vel(1); q0(8)  = angular_vel(2);
    q0(9)  = com_vel(0);     q0(10) = com_vel(1);     q0(11) = com_vel(2);
    q0(12) = dummy_tactile_left(0);  q0(13) = dummy_tactile_left(1);  q0(14) = dummy_tactile_left(2);
    q0(15) = dummy_tactile_right(0); q0(16) = dummy_tactile_right(1); q0(17) = dummy_tactile_right(2);

    // Reference: desired yaw, foot-projected CoM position, nominal contact widths
    Vector3d pos_base = human_position + human_rotation_matrix * L;
    X_ref(0) = euler_angle_d(0); X_ref(1) = euler_angle_d(1); X_ref(2) = euler_angle_d(2);
    X_ref(3) = pos_base(0);      X_ref(4) = pos_base(1);      X_ref(5) = pos_base(2) + c/2;

    std::cout << "q0:    " << q0    << std::endl;
    std::cout << "X_ref: " << X_ref << std::endl;
}

void OptimalTrajectory::centroidalNMPC()
{
    using namespace casadi;

    // Update parameters and re-solve; the symbolic graph is not rebuilt
    nmpc_.set_value(q0_param_,    q0);
    nmpc_.set_value(X_ref_param_, X_ref);

    auto t_mpc_start = std::chrono::steady_clock::now();
    OptiSol sol = nmpc_.solve();
    auto t_mpc_end = std::chrono::steady_clock::now();
    mpc_solve_time_ms_ = std::chrono::duration<double, std::milli>(t_mpc_end - t_mpc_start).count();

    DM f_all   = sol.value(F_var_);
    DM f_first = f_all(Slice(), 0);
    std::cout << "solution: " << f_first << std::endl;

    force_left  << static_cast<double>(f_first(0).scalar()),
                   static_cast<double>(f_first(1).scalar()),
                   static_cast<double>(f_first(2).scalar());
    force_right << static_cast<double>(f_first(3).scalar()),
                   static_cast<double>(f_first(4).scalar()),
                   static_cast<double>(f_first(5).scalar());
    dpos_left   << static_cast<double>(f_first(6).scalar()),
                   static_cast<double>(f_first(7).scalar()),
                   static_cast<double>(f_first(8).scalar());
    dpos_right  << static_cast<double>(f_first(9).scalar()),
                   static_cast<double>(f_first(10).scalar()),
                   static_cast<double>(f_first(11).scalar());
}

casadi::MX OptimalTrajectory::centroidalDynamics(const casadi::MX& current_state, const casadi::MX& input)
{
    // Unpack state
    casadi::MX theta = current_state(casadi::Slice(0,  3));
    casadi::MX p     = current_state(casadi::Slice(3,  6));
    casadi::MX omega = current_state(casadi::Slice(6,  9));
    casadi::MX v     = current_state(casadi::Slice(9,  12));
    casadi::MX p1    = current_state(casadi::Slice(12, 15));
    casadi::MX p2    = current_state(casadi::Slice(15, 18));

    // Unpack input
    casadi::MX f1  = input(casadi::Slice(0, 3));
    casadi::MX f2  = input(casadi::Slice(3, 6));
    casadi::MX dp1 = input(casadi::Slice(6, 9));
    casadi::MX dp2 = input(casadi::Slice(9, 12));

    casadi::MX z = theta(0), y = theta(1), x = theta(2);

    // ZYX Euler kinematics matrix T: dtheta = T * omega
    casadi::MX T = casadi::MX::zeros(3, 3);
    T(0,1) = -sin(z);       T(0,2) = cos(z)*cos(y);
    T(1,1) =  cos(z);       T(1,2) = cos(y)*sin(z);
    T(2,0) =  1;             T(2,2) = -sin(y);

    // ZYX rotation matrix R = Rz * Ry * Rx
    casadi::MX Rz = casadi::MX::zeros(3,3), Ry = casadi::MX::zeros(3,3), Rx = casadi::MX::zeros(3,3);
    Rz(0,0) = cos(z); Rz(0,1) = -sin(z); Rz(1,0) = sin(z); Rz(1,1) = cos(z); Rz(2,2) = 1;
    Ry(0,0) = cos(y); Ry(0,2) =  sin(y); Ry(1,1) = 1; Ry(2,0) = -sin(y); Ry(2,2) = cos(y);
    Rx(0,0) = 1; Rx(1,1) = cos(x); Rx(1,2) = -sin(x); Rx(2,1) = sin(x); Rx(2,2) = cos(x);
    casadi::MX R = Rz * Ry * Rx;

    casadi::MX N      = groundReactionForce(theta, omega);
    casadi::MX dtheta = mtimes(T, omega);
    casadi::MX dp_com = mtimes(R, v);
    casadi::MX domega = mtimes(inertial_matrix_inv,
                               -cross(omega, mtimes(inertial_matrix, omega))
                               + cross(p1, f1) + cross(p2, f2) + cross(L_foot_com, N));
    casadi::MX dv     = (f1 + f2 + N + mtimes(transpose(R), m * g * casadi::MX::vertcat({0, 0, 1}))) / m;

    casadi::MX dX = casadi::MX::zeros(18);
    dX(casadi::Slice(0,  3))  = dtheta;
    dX(casadi::Slice(3,  6))  = dp_com;
    dX(casadi::Slice(6,  9))  = domega;
    dX(casadi::Slice(9,  12)) = dv;
    dX(casadi::Slice(12, 15)) = dp1;
    dX(casadi::Slice(15, 18)) = dp2;
    return dX;
}

void OptimalTrajectory::publishCommand()
{
    trajectory_msg.force_left.x  = force_left[0];
    trajectory_msg.force_left.y  = force_left[1];
    trajectory_msg.force_left.z  = force_left[2];
    trajectory_msg.force_right.x = force_right[0];
    trajectory_msg.force_right.y = force_right[1];
    trajectory_msg.force_right.z = force_right[2];
    trajectory_msg.dp_left.x  = dummy_tactile_left_world[0];
    trajectory_msg.dp_left.y  = dummy_tactile_left_world[1];
    trajectory_msg.dp_left.z  = dummy_tactile_left_world[2];
    trajectory_msg.dp_right.x = dummy_tactile_right_world[0];
    trajectory_msg.dp_right.y = dummy_tactile_right_world[1];
    trajectory_msg.dp_right.z = dummy_tactile_right_world[2];

    dummy_tactile_left  += dpos_left  * loop_time;
    dummy_tactile_right += dpos_right * loop_time;

    traj_pub.publish(trajectory_msg);
}

void OptimalTrajectory::publishTactileMarker()
{
    dummy_tactile_left_world  = human_rotation_matrix * dummy_tactile_left  + human_position;
    dummy_tactile_right_world = human_rotation_matrix * dummy_tactile_right + human_position;

    auto fillSphere = [](visualization_msgs::Marker& m, int id,
                         double x, double y, double z) {
        m.id     = id;
        m.type   = visualization_msgs::Marker::SPHERE;
        m.ns     = "optimal_traj";
        m.header.frame_id = "world";
        m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z;
        m.pose.orientation.w = 0.0;
        m.scale.x = m.scale.y = m.scale.z = 0.2;
        m.color.a = 0.8; m.color.r = 0.4; m.color.g = 0.4;
    };
    fillSphere(tactile_marker_left,  0,
               dummy_tactile_left_world[0],  dummy_tactile_left_world[1],  dummy_tactile_left_world[2]);
    fillSphere(tactile_marker_right, 1,
               dummy_tactile_right_world[0], dummy_tactile_right_world[1], dummy_tactile_right_world[2]);

    Eigen::Quaterniond quaternion(rotation_matrix_d);
    std::cout << "rotation_matrix_d:\n" << rotation_matrix_d << std::endl;

    desired_marker.id   = 2;
    desired_marker.type = visualization_msgs::Marker::CUBE;
    desired_marker.ns   = "optimal_traj";
    desired_marker.header.frame_id = "world";
    desired_marker.pose.position.x = human_position[0];
    desired_marker.pose.position.y = human_position[1];
    desired_marker.pose.position.z = human_position[2];
    desired_marker.pose.orientation.x = quaternion.x();
    desired_marker.pose.orientation.y = quaternion.y();
    desired_marker.pose.orientation.z = quaternion.z();
    desired_marker.pose.orientation.w = quaternion.w();
    desired_marker.scale.x = 0.2; desired_marker.scale.y = 0.4; desired_marker.scale.z = 1.8;
    desired_marker.color.a = 0.7; desired_marker.color.b = 1.0;

    transform_stamped.header.stamp    = ros::Time::now();
    transform_stamped.header.frame_id = "world";
    transform_stamped.child_frame_id  = 3;
    transform_stamped.transform.translation.x = human_position[0];
    transform_stamped.transform.translation.y = human_position[1];
    transform_stamped.transform.translation.z = human_position[2];
    transform_stamped.transform.rotation.x = quaternion.x();
    transform_stamped.transform.rotation.y = quaternion.x();  // NOTE: likely a bug in original (y = x)
    transform_stamped.transform.rotation.z = quaternion.z();
    transform_stamped.transform.rotation.w = quaternion.w();
    tf_broadcaster.sendTransform(transform_stamped);

    marker_pub.publish(tactile_marker_right);
    marker_pub.publish(tactile_marker_left);
    marker_pub.publish(desired_marker);
}

void OptimalTrajectory::spin()
{
    while (!start_flag && ros::ok()) {
        ROS_WARN_STREAM("Waiting for intent detection");
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO_STREAM("Intent detected, program starts");

    double sum_time = 0.0;
    int    count    = 0;
    while (ros::ok()) {
        ros::spinOnce();
        double t1 = ros::Time::now().toSec();

        getHumanPose();
        centroidalNMPC();
        publishTactileMarker();
        publishCommand();

        double t2 = ros::Time::now().toSec() - t1;
        sum_time += 1000 * t2;
        count    += 1;
        std::cout << "average time: " << sum_time / count << " ms\n" << std::endl;

        if (pose_changed_) {
            std::ofstream f(perf_log_path_, std::ios::app);
            f << "| " << std::fixed << std::setprecision(3) << ros::Time::now().toSec()
              << " | pose_change | " << mpc_solve_time_ms_
              << " | - | - |\n";
            f.close();
            pose_changed_ = false;
        }

        loop_rate.sleep();
    }
}
