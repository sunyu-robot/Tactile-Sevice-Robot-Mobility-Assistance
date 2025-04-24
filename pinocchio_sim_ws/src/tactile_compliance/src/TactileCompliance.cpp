/*
 * Copyright (c) 2023 Yu Sun <sunyuu@zju.edu.cn>
 * All rights reserved.
 */
#include "tactile_compliance/TactileCompliance.hpp"

TactileCompliance::TactileCompliance()
{
    for(int i=0;i<2;i++){
        skinstates[i].x_error.setZero();
        skinstates[i].dx_error.setZero();
        skinstates[i].x_error_rotation.setZero();
        skinstates[i].dx_error_rotation.setZero();
        skinstates[i].ddx_desire_skin.setZero();
        skinstates[i].dx_desire_skin.setZero();
        skinstates[i].x_desire_skin.setZero();
        skinstates[i].zmp_vector_local << (double)(10) * 0.015, 0 ,0.039;
        skinstates[i].desire_zmp_local << (double)(10) * 0.015, 0 ,0.039;
        skinstates[i].initialized = false;
    }

    elapsed_sec = 0;

    // rotation
    vector<double> _Mxinv_rotate;
    if(!nh.getParam("MXROTATE",_Mxinv_rotate))
    {
        ROS_ERROR_STREAM("can't get Mx_rotate");
        ros::shutdown();
    }
    Matrix3d _Mx_eigen_inv_rotate(_Mxinv_rotate.data());
    Mxinv_rotate = _Mx_eigen_inv_rotate.transpose();

    vector<double> _Bx_rotate;
    if(!nh.getParam("BXROTATE",_Bx_rotate))
    {
        ROS_ERROR_STREAM("can't get Bx_rotate");
        ros::shutdown();
    }
    Matrix3d _Bx_eigen_rotate(_Bx_rotate.data());
    Bx_rotate = _Bx_eigen_rotate.transpose();

    vector<double> _Kx_rotate;
    if(!nh.getParam("KXROTATE",_Kx_rotate))
    {
        ROS_ERROR_STREAM("can't get Kx_rotate");
        ros::shutdown();
    }
    Matrix3d _Kx_eigen_rotate(_Kx_rotate.data());
    Kx_rotate = _Kx_eigen_rotate.transpose();

    trajectory_sub = nh.subscribe("trajectory", 3 ,&TactileCompliance::trajectoryCallback ,this);

    userpose_sub = nh.subscribe("/dummy_human_pose",10, &TactileCompliance::poseCallback, this);

    contact_left_pub = nh.advertise<geometry_msgs::Vector3>("/contact_position_left", 10);
    contact_right_pub = nh.advertise<geometry_msgs::Vector3>("/contact_position_right", 10);

    constriant_matrix.resize(815,15);

    Kp = 4; Kd = 0.8;

    human_rotation_matrix.setIdentity();
}

void TactileCompliance::poseCallback(const geometry_msgs::PoseStampedConstPtr &input)
{ 
    human_position << input->pose.position.x, input->pose.position.y, input->pose.position.z;  
    Eigen::Quaterniond quaternion(input->pose.orientation.w, input->pose.orientation.x, input->pose.orientation.y, input->pose.orientation.z);
    human_rotation_matrix = quaternion.toRotationMatrix();
    // euler_angle = human_rotation_matrix.eulerAngles(2, 1, 0);
}

void TactileCompliance::trajectoryCallback(const util_msgs::trajectoryConstPtr &input)
{
    // force (in body frame)
    force_left << input->force_left.x, input->force_left.y, input->force_left.z;
    force_right << input->force_right.x, input->force_right.y, input->force_right.z;
    dp_left << input->dp_left.x, input->dp_left.y, input->dp_left.z;
    dp_right << input->dp_right.x, input->dp_right.y, input->dp_right.z;
}

void TactileCompliance::setInitVariable(const Matrix3d _Mxinv, const Matrix3d _Bx, const Matrix3d _Kx, const double _loop_time)
{
    Mxinv = _Mxinv;
    Bx = _Bx;
    Kx = _Kx;
    loop_time = _loop_time;
}

void TactileCompliance::publishContact()
{
    contact_position_left_msg.x = skinstates[0].x_real_skin[0];
    contact_position_left_msg.y = skinstates[0].x_real_skin[1];
    contact_position_left_msg.z = skinstates[0].x_real_skin[2];
    contact_position_right_msg.x = skinstates[1].x_real_skin[0];
    contact_position_right_msg.y = skinstates[1].x_real_skin[1];
    contact_position_right_msg.z = skinstates[1].x_real_skin[2];

    contact_left_pub.publish(contact_position_left_msg);
    contact_right_pub.publish(contact_position_right_msg);
}

void TactileCompliance::tactileTrack(const Vector15d q_real, const Vector15d dq_real, Vector15d& gradient_vector, Matrix15d& Heq, Vector15d& dq_cmd)
{
    gradient_vector.setZero();
    Heq.setZero();

    for(int i=0;i<2;i++)
    {
        if(elapsed_sec < 5)
            skinstates[0].torque << 0,0,0;
        else       
            skinstates[0].torque.setZero();
        if(elapsed_sec < 5){
            skinstates[0].force << 0,0,0;
            skinstates[1].force << -0,0,0;
        }
        else{
            skinstates[0].force.setZero();
            skinstates[1].force.setZero();
        }
        // joint id 
        skinstates[i].joint_id = (int) (skinstates[i].zmp_vector_local[0] /0.015);
        // skinstates[i].joint_id = 10;
        // jacobian & rotation matrix
        skinstates[i].jacobian_matrix_linear = linearJacobianFuncArray[i](skinstates[i].joint_id,q_real(2),q_real(3+6*(i%2)),q_real(4+6*(i%2)),q_real(5+6*(i%2)),q_real(6+6*(i%2)),q_real(7+6*(i%2)),q_real(8+6*(i%2)));
        skinstates[i].rotation_matrix = rotationFuncArray[i](q_real(2),q_real(3+6*(i%2)),q_real(4+6*(i%2)),q_real(5+6*(i%2)));
        // traj force
        if(i == 0){
            skinstates[0].force += human_rotation_matrix * force_left*0.2;
        }
        else if(i == 1){
            skinstates[1].force += human_rotation_matrix * force_right*0.2;
        }

        // compute cartesian space motion
        skinstates[i].ddx_error = Mxinv * (skinstates[i].force + skinstates[i].force_hip - Bx * skinstates[i].dx_error - Kx*skinstates[i].x_error);
        skinstates[i].dx_error += skinstates[i].ddx_error*loop_time;
        skinstates[i].x_error += skinstates[i].dx_error*loop_time - skinstates[i].ddx_error*loop_time*loop_time/2;

        // position
        skinstates[i].zmp_vector = skinstates[i].rotation_matrix * skinstates[i].zmp_vector_local;
        skinstates[i].start_position = jointPosFuncArray[i](q_real(0),q_real(1),q_real(2),q_real(3+6*(i%2)),q_real(4+6*(i%2)),q_real(5+6*(i%2)),q_real(6+6*(i%2)),q_real(7+6*(i%2)));
        skinstates[i].x_real_skin = skinstates[i].start_position + skinstates[i].zmp_vector;

        if(skinstates[i].initialized == false){
            skinstates[i].initialized = true;
            start_time = ros::Time::now();
            skinstates[i].rotation_matrix_init = skinstates[i].rotation_matrix;
            skinstates[i].rotation_matrix_d = skinstates[i].rotation_matrix_init;
            skinstates[i].start_position_init = skinstates[i].start_position;
            dp_left = skinstates[0].x_real_skin;
            dp_right = skinstates[1].x_real_skin;
        }
        else{
            current_time = ros::Time::now();
            elapsed_time = current_time - start_time;
            elapsed_sec = elapsed_time.toSec();
        }

        skinstates[i].desire_zmp_world = skinstates[i].start_position + skinstates[i].rotation_matrix * skinstates[i].desire_zmp_local;
        skinstates[i].x_desire_skin_zmp = skinstates[i].start_position_init + skinstates[i].rotation_matrix_init * skinstates[i].zmp_vector_local;
        skinstates[i].x_zc = skinstates[i].x_real_skin - skinstates[i].desire_zmp_world;
        
        skinstates[i].x_desire_skin = skinstates[i].x_desire_skin_zmp + skinstates[i].x_zc;
        if(i == 0){
            skinstates[i].x_desire_skin = dp_left;
            skinstates[i].rotation_matrix_d = human_rotation_matrix * skinstates[i].rotation_matrix_init;
        }
        else if(i == 1){
            skinstates[i].x_desire_skin = dp_right;
            skinstates[i].rotation_matrix_d = human_rotation_matrix * skinstates[i].rotation_matrix_init;
        }
        // traj dp

        skinstates[i].ddx_newdesire_skin = skinstates[i].ddx_error + skinstates[i].ddx_desire_skin;
        skinstates[i].dx_newdesire_skin = skinstates[i].dx_error + skinstates[i].dx_desire_skin;
        skinstates[i].x_newdesire_skin = skinstates[i].x_error + skinstates[i].x_desire_skin;
        skinstates[i].dx_real_skin = skinstates[i].jacobian_matrix_linear * dq_real;
        skinstates[i].x_track = skinstates[i].x_newdesire_skin - skinstates[i].x_real_skin + skinstates[i].dx_newdesire_skin;
        // if(i == 0){
        //     skinstates[0].dx_desire_skin = dp_left;
        //     dp_left_accu += dp_left * loop_time;
        //     skinstates[i].x_track += dp_left_accu;
        // }
        // else if(i == 1){
        //     skinstates[1].dx_desire_skin = dp_right;
        //     dp_right_accu += dp_right * loop_time;
        //     skinstates[i].x_track += dp_right_accu;
        // }

        skinstates[i].dx_track = skinstates[i].x_track * Kp  + ( - skinstates[i].dx_real_skin) * Kd;
        std::cout << "skinstates[" << i << "].dx_newdesire_skin" << skinstates[i].dx_newdesire_skin.transpose() << std::endl;
        std::cout << "skinstates[" << i << "].x_zc" << skinstates[i].x_zc.transpose() << std::endl;
        std::cout << "skinstates[" << i << "].dx_track" << skinstates[i].dx_track.transpose() << std::endl;
        std::cout << "skinstates[" << i << "].x_track" << skinstates[i].x_track.transpose() << std::endl;
        // ros::shutdown();

        // for rotation
        // jacobian matrix
        skinstates[i].jacobian_matrix_angular = angularJacobianFuncArray[i](q_real(2),q_real(3+6*(i%2)),q_real(4+6*(i%2)),q_real(5+6*(i%2)),q_real(6+6*(i%2)),q_real(7+6*(i%2)),q_real(8+6*(i%2)));
        // cartesian space rotation
        skinstates[i].ddx_error_rotation = Mxinv_rotate * (skinstates[i].torque - Bx_rotate * skinstates[i].dx_error_rotation - Kx_rotate*skinstates[i].x_error_rotation);
        skinstates[i].dx_error_rotation += skinstates[i].ddx_error_rotation*loop_time;
        // rotation matrix approch

        // skinstates[i].so3_rotation = Sophus::SO3d::exp(skinstates[i].dx_error_rotation * loop_time - skinstates[i].ddx_error_rotation*loop_time*loop_time/2);
        // skinstates[i].so3_rotation_current = Sophus::SO3d::exp(skinstates[i].x_error_rotation);
        auto so3_rotation = expMap(skinstates[i].dx_error_rotation * loop_time - skinstates[i].ddx_error_rotation*loop_time*loop_time/2);
        auto so3_rotation_current = expMap(skinstates[i].x_error_rotation);
        // skinstates[i].x_error_rotation_matrix = skinstates[i].so3_rotation.matrix() * skinstates[i].so3_rotation_current.matrix();
        skinstates[i].x_error_rotation_matrix = so3_rotation * so3_rotation_current;
        // Sophus::SO3d so3_next(skinstates[i].x_error_rotation_matrix);
        // skinstates[i].x_error_rotation = so3_next.log();
        skinstates[i].x_error_rotation = logMap(skinstates[i].x_error_rotation_matrix);
        // get weight
        skinstates[i].weight = 10;
        sum_weight += skinstates[i].weight;

        // rotation 
        skinstates[i].rotation_matrix_d_new = skinstates[i].x_error_rotation_matrix * skinstates[i].rotation_matrix_d;
        skinstates[i].track_rotation_matrix = skinstates[i].rotation_matrix_d_new * skinstates[i].rotation_matrix.transpose();
        // Sophus::SO3d so3_track(skinstates[i].track_rotation_matrix);
        // skinstates[i].so3_log = so3_track.log();
        skinstates[i].so3_log = logMap(skinstates[i].track_rotation_matrix);

        // // compute hessian matrix and linear vector
        // Heq += skinstates[i].weight*skinstates[i].jacobian_matrix_linear.transpose() * skinstates[i].jacobian_matrix_linear*10*0.3;       
        // gradient_vector += -skinstates[i].weight * skinstates[i].x_track.transpose() * skinstates[i].jacobian_matrix_linear*10;

        Eigen::Matrix3d Q_rotate;
        Q_rotate.setIdentity();
        Q_rotate(0, 0) = 0.00001;
        Q_rotate(1, 1) = 0.00001;

        Heq += skinstates[i].weight*skinstates[i].jacobian_matrix_linear.transpose() * skinstates[i].jacobian_matrix_linear;       
        gradient_vector += -skinstates[i].weight * skinstates[i].dx_track.transpose() * skinstates[i].jacobian_matrix_linear;

        Heq += skinstates[i].weight*skinstates[i].jacobian_matrix_angular.transpose()*Q_rotate * skinstates[i].jacobian_matrix_angular*0.05*10;      
        gradient_vector += -skinstates[i].weight* skinstates[i].so3_log.transpose()*Q_rotate  * skinstates[i].jacobian_matrix_angular*10;
    }
    publishContact();
}

void TactileCompliance::tactileRotation(const Vector15d dq_solution_first, Vector15d& gradient_vector, Matrix15d& Heq, Eigen::SparseMatrix<double> &LinearConstraintsMatrix,Matrix<double, 815, 1> &lowerBound,Matrix<double, 815, 1> &upperBound)
{
    // not used
    gradient_vector.setZero();
    Heq.setZero();
    constriant_matrix = LinearConstraintsMatrix.toDense();

    for(int i=0;i<4;i++)
    {
        Heq += skinstates[i].weight*skinstates[i].jacobian_matrix_angular.transpose()*skinstates[i].jacobian_matrix_angular;       
        gradient_vector += -skinstates[i].weight * skinstates[i].dx_error_rotation.transpose() * skinstates[i].jacobian_matrix_angular;

        // Heq += skinstates[i].weight*skinstates[i].jacobian_matrix_angular.transpose()*skinstates[i].jacobian_matrix_angular*0.05;       
        // gradient_vector += -skinstates[i].weight * skinstates[i].x_error_rotation.transpose() * skinstates[i].jacobian_matrix_angular;

        // for Constraints
        constriant_matrix.block(3*i,0,3,15) = skinstates[i].weight*skinstates[i].jacobian_matrix_linear*10;
        lowerBound.block(3*i,0,3,1) = skinstates[i].weight*skinstates[i].jacobian_matrix_linear*dq_solution_first*10;
        upperBound.block(3*i,0,3,1) = skinstates[i].weight*skinstates[i].jacobian_matrix_linear*dq_solution_first*10;
    }
    LinearConstraintsMatrix = constriant_matrix.sparseView();
}

void TactileCompliance::tactileMarker(visualization_msgs::MarkerArray &marker_array)
{
    for(int i = 0;i<4;i++)
    {
        // cylinder marker
        skinstates[i].cylinder.id = 4+i;
        skinstates[i].cylinder.ns = "cylindar";
        skinstates[i].cylinder.type = visualization_msgs::Marker::CYLINDER;
        skinstates[i].cylinder.pose.orientation.x = 0.7071;  
        skinstates[i].cylinder.pose.orientation.y = 0;
        skinstates[i].cylinder.pose.orientation.z = -0.7071;
        skinstates[i].cylinder.pose.orientation.w = 0;
        skinstates[i].cylinder.color.a = 1;  // 不透明度
        skinstates[i].cylinder.color.r = skinstates[i].weight * 0.2;  // 交互时为红色
        skinstates[i].cylinder.color.b = 1.0-skinstates[i].weight * 0.2;  // 不交互时为蓝色

        if(i == 0)
            skinstates[i].cylinder.header.frame_id = "L_forearm_link";
        else if(i == 1)
            skinstates[i].cylinder.header.frame_id = "R_forearm_link";
        else if(i == 2)
            skinstates[i].cylinder.header.frame_id = "L_upper_arm_link";
        else if(i == 3)
            skinstates[i].cylinder.header.frame_id = "R_upper_arm_link";

        if(i<2){
            skinstates[i].cylinder.pose.position.x = -0.125 - 0.093;  
            skinstates[i].cylinder.pose.position.y = 0.0;
            skinstates[i].cylinder.pose.position.z = 0.039;
            skinstates[i].cylinder.scale.x = 0.1;  
            skinstates[i].cylinder.scale.y = 0.1;
            skinstates[i].cylinder.scale.z = 0.25;
        }
        else{
            skinstates[i].cylinder.pose.position.x = -0.125 - 0.175;  
            skinstates[i].cylinder.pose.position.y = 0.0;
            skinstates[i].cylinder.pose.position.z = 0.1807;
            skinstates[i].cylinder.scale.x = 0.13;  
            skinstates[i].cylinder.scale.y = 0.13;
            skinstates[i].cylinder.scale.z = 0.25;
        }
        marker_array.markers.push_back(skinstates[i].cylinder);

        // arrow marker
        skinstates[i].arrow.id = 6+i;
        skinstates[i].arrow.ns = "force arrow";
        skinstates[i].arrow.type = visualization_msgs::Marker::ARROW;
        skinstates[i].arrow.action = visualization_msgs::Marker::ADD;
        // skinstates[i].arrow.lifetime = ros::Duration(0.01);
        skinstates[i].arrow.frame_locked = true;
        skinstates[i].arrow.scale.x = 0.04;
        skinstates[i].arrow.scale.y = 0.09;
        skinstates[i].arrow.scale.z = 0.047;
        skinstates[i].arrow.header.frame_id = "world";  
        skinstates[i].arrow.header.stamp = ros::Time::now();
        skinstates[i].arrow.color.r = 0.5;
        skinstates[i].arrow.color.g = 0.5;
        skinstates[i].arrow.color.b = 0.0;
        skinstates[i].arrow.color.a = 0.9;
        // arrow origin
        skinstates[i].start_point.x = skinstates[i].start_position[0] + skinstates[i].zmp_vector[0];
        skinstates[i].start_point.y = skinstates[i].start_position[1] + skinstates[i].zmp_vector[1];
        skinstates[i].start_point.z = skinstates[i].start_position[2] + skinstates[i].zmp_vector[2];
        // arrow direction
        skinstates[i].direction.x = skinstates[i].force(0) * 0.4;
        skinstates[i].direction.y = skinstates[i].force(1) * 0.4;
        skinstates[i].direction.z = skinstates[i].force(2) * 0.4;
        // arrow end
        skinstates[i].end_point.x = skinstates[i].start_point.x + skinstates[i].direction.x;
        skinstates[i].end_point.y = skinstates[i].start_point.y + skinstates[i].direction.y;
        skinstates[i].end_point.z = skinstates[i].start_point.z + skinstates[i].direction.z;
        // set marker
        skinstates[i].arrow.points.push_back(skinstates[i].start_point);
        skinstates[i].arrow.points.push_back(skinstates[i].end_point);

        marker_array.markers.push_back(skinstates[i].arrow);
        skinstates[i].arrow.points.clear();
    }
}

Eigen::Matrix3d TactileCompliance::expMap(const Eigen::Vector3d& omega){
    double theta = omega.norm();
    if(theta < 1e-10){
        return Eigen::Matrix3d::Identity();
    }

    Eigen::Vector3d axis = omega / theta;
    Eigen::AngleAxisd angleAxis(theta, axis);

    return angleAxis.toRotationMatrix();
}

Eigen::Vector3d TactileCompliance::logMap(const Eigen::Matrix3d& rotation_matrix){
    Eigen::AngleAxisd angleAxis(rotation_matrix);

    return angleAxis.angle() * angleAxis.axis();
}