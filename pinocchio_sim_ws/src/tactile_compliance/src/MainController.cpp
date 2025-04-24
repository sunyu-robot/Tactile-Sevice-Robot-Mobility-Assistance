/*
 * Copyright (c) 2023 Yu Sun <sunyuu@zju.edu.cn>
 * All rights reserved.
 */
#include <tactile_compliance/MainController.hpp>

void Tactile_HQP::init(){
    // ur16e joints' limit
    if(!nh.getParam("JOINT_LIMIT", q_limit)){
        ROS_ERROR_STREAM("can't get joints limit");
        ros::shutdown();
    }
    /// Get velocity limits
    if(!nh.getParam("VELOCITY_LIMIT", velocity_limit)){
        ROS_ERROR_STREAM("can't get velocity limit");
        ros::shutdown();
    }

    vector<double> _init_left_joints_position;
    if(!nh.getParam("INIT_LEFT_JOINTS_POSITION", _init_left_joints_position)){
        ROS_ERROR_STREAM("can't get init left joints position!");
        ros::shutdown();
    }

    vector<double> _init_right_joints_position;
    if(!nh.getParam("INIT_RIGHT_JOINTS_POSITION", _init_right_joints_position)){
        ROS_ERROR_STREAM("can't get init right joints position!");
        ros::shutdown();
    }
    init_joints_pose << 0,0,0,
                    Map<Vector6d>(_init_left_joints_position.data()),
                    Map<Vector6d>(_init_right_joints_position.data());

    vector<double> _Mx;
    if(!nh.getParam("MX",_Mx))
    {
        ROS_ERROR_STREAM("can't get Mx");
        ros::shutdown();
    }
    Matrix3d _Mx_eigen(_Mx.data());

    vector<double> _Bx;
    if(!nh.getParam("BX",_Bx))
    {
        ROS_ERROR_STREAM("can't get Bx");
        ros::shutdown();
    }
    Matrix3d _Bx_eigen(_Bx.data());

    vector<double> _Kx;
    if(!nh.getParam("KX",_Kx))
    {
        ROS_ERROR_STREAM("can't get Kx");
        ros::shutdown();
    }
    Matrix3d _Kx_eigen(_Kx.data());

    dq_cmd.setZero();
    joint_state_sub = nh.subscribe("/joint_states" ,3 ,&Tactile_HQP::jointstateCallback ,this);
    dq_pub[0] = nh.advertise<std_msgs::Float64>("/x_dir_controller/command",1);
    dq_pub[1] = nh.advertise<std_msgs::Float64>("/y_dir_controller/command",1);
    dq_pub[2] = nh.advertise<std_msgs::Float64>("/z_dir_controller/command",1);
    dq_pub[3] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_shoulder_pan_joint/command",1);
    dq_pub[4] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_shoulder_lift_joint/command",1);
    dq_pub[5] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_elbow_joint/command",1);
    dq_pub[6] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_wrist_1_joint/command",1);
    dq_pub[7] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_wrist_2_joint/command",1);
    dq_pub[8] = nh.advertise<std_msgs::Float64>("/L_arm_controller/L_wrist_3_joint/command",1);
    dq_pub[9] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_shoulder_pan_joint/command",1);
    dq_pub[10] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_shoulder_lift_joint/command",1);
    dq_pub[11] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_elbow_joint/command",1);
    dq_pub[12] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_wrist_1_joint/command",1);
    dq_pub[13] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_wrist_2_joint/command",1);
    dq_pub[14] = nh.advertise<std_msgs::Float64>("/R_arm_controller/R_wrist_3_joint/command",1);
    
    marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

    loop_time = 1.0/LOOP_RATE;
    sum_time = 0;
    count = 0;
    timer = ros::Time::now().toSec();

    // set value for tactile compliance
    tactile_compliance.setInitVariable(_Mx_eigen,_Bx_eigen,_Kx_eigen,loop_time);

    // OSQP initialization
    gradient_first.resize(15);
    hessian_first.resize(15,15);
    LinearConstraintsMatrix.resize(815,15);
    // LinearConstraintsMatrix.bottomRightCorner(15, 15) = Eigen::MatrixXd::Identity(15, 15);
    lowerBound.resize(815);
    upperBound.resize(815);
    I15d.setIdentity();
    for (int i = 0; i<815;i++)
    {
        lowerBound[i] = -1;
        upperBound[i] = 1;
    }

    // easily to cause core dumped
    solver.settings()->setWarmStart(false);
    solver.data()->setNumberOfVariables(15);
    solver.data()->setNumberOfConstraints(815);
    if(!solver.data()->setHessianMatrix(hessian_first)) ros::shutdown();
    if(!solver.data()->setGradient(gradient_first)) ros::shutdown();
    if(!solver.data()->setLinearConstraintsMatrix(LinearConstraintsMatrix)) ros::shutdown();
    if(!solver.data()->setLowerBound(lowerBound)) ros::shutdown();
    if(!solver.data()->setUpperBound(upperBound)) ros::shutdown();
    if(!solver.initSolver()) ros::shutdown();

    // critical point initialization
    critical_point.initsphere(point_sphere);
}

void Tactile_HQP::jointstateCallback(const sensor_msgs::JointStateConstPtr &input){
    q_real << input->position[12],
              input->position[13],
              input->position[14],
              input->position[2],
              input->position[1],
              input->position[0],
              input->position[3],
              input->position[4],
              input->position[5],
              input->position[8],
              input->position[7],
              input->position[6],
              input->position[9],
              input->position[10],
              input->position[11];

    dq_real <<input->velocity[12],
              input->velocity[13],
              input->velocity[14],
              input->velocity[2],
              input->velocity[1],
              input->velocity[0],
              input->velocity[3],
              input->velocity[4],
              input->velocity[5],
              input->velocity[8],
              input->velocity[7],
              input->velocity[6],
              input->velocity[9],
              input->velocity[10],
              input->velocity[11];

    timer = input->header.stamp.toSec();
}

void Tactile_HQP::robotInitPose(const Vector15d& input)
{
    ros::spinOnce();
    Vector15d err = input - q_real;
    while(err.norm() > 0.01){
        ros::spinOnce();
        dq_cmd = 0.5*err;
        publishCommand();
        sleep(0.2);
        err = input - q_real;
    }
}

void Tactile_HQP::markerVisualization(){
    tactile_compliance.tactileMarker(marker_array);
    critical_point.pointMarker(marker_array, point_sphere);
    marker_pub.publish(marker_array);
    marker_array.markers.clear();
}

void Tactile_HQP::getCommand(){
    // update collision constraint
    critical_point.updatedata(q_real, point_sphere);

    // first layer QP
    tactile_compliance.tactileTrack(q_real, dq_real, gradient_vector, Heq, dq_cmd);
    hessian = Heq + 0.1*I15d;
    hessian_first= hessian.sparseView();
    gradient_first = gradient_vector.sparseView();

    collision_constraint.updateConstraintsMatrix(10, point_sphere, LinearConstraintsMatrix, lowerBound, upperBound, q_real);

    solver.updateHessianMatrix(hessian_first);
    solver.updateGradient(gradient_first);
    solver.updateLinearConstraintsMatrix(LinearConstraintsMatrix);
    solver.updateLowerBound(lowerBound);
    solver.updateUpperBound(upperBound);
    if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) ros::shutdown();
    OsqpEigen::Status status = solver.getStatus();
    if(status == OsqpEigen::Status::Solved)
        dq_solution_first = solver.getSolution();
    else if(status == OsqpEigen::Status::DualInfeasible)
        ROS_ERROR("Solver DualInfeasible.");
    else if(status == OsqpEigen::Status::SolvedInaccurate)
        ROS_ERROR("Solver SolvedInaccurate.");
    else if(status == OsqpEigen::Status::NonCvx)
        ROS_ERROR("Solver Non convex.");
    else if(status == OsqpEigen::Status::PrimalInfeasible)
    {
        ROS_ERROR("Solver PrimalInfeasible.");
        dq_solution_first.setZero();
    }
    else if(status == OsqpEigen::Status::PrimalInfeasibleInaccurate)
        ROS_ERROR("Solver PrimalInfeasibleInaccurate.");

    
    // if(dq_solution_first.norm()>0.001 && count > 100)
    // {
    //     std::cout << "hessian :" << hessian << std::endl;
    //     std::cout << "gradient_vector :" << gradient_vector.transpose() << std::endl;
    //     // ros::shutdown();
    // }

    // second layer QP
    // tactile_compliance.tactileRotation(dq_solution_first, gradient_vector, Heq, LinearConstraintsMatrix, lowerBound, upperBound);
    // hessian = Heq + 0.1*I15d;
    // hessian_second= hessian.sparseView();
    // gradient_second = gradient_vector.sparseView();
    // solver.updateHessianMatrix(hessian_second);
    // solver.updateGradient(gradient_second);
    // solver.updateLinearConstraintsMatrix(LinearConstraintsMatrix);
    // solver.updateLowerBound(lowerBound);
    // solver.updateUpperBound(upperBound);
    // if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) ros::shutdown();
    // status = solver.getStatus();
    // if(status == OsqpEigen::Status::Solved)
    //     dq_solution_second = solver.getSolution();
    // else if(status == OsqpEigen::Status::DualInfeasible)
    //     ROS_ERROR("Solver DualInfeasible.");
    // else if(status == OsqpEigen::Status::SolvedInaccurate)
    //     ROS_ERROR("Solver SolvedInaccurate.");
    // else if(status == OsqpEigen::Status::NonCvx)
    //     ROS_ERROR("Solver Non convex.");
    // else if(status == OsqpEigen::Status::PrimalInfeasible)
    // {
    //     ROS_ERROR("Solver PrimalInfeasible.");
    //     dq_solution_second.setZero();
    // }
    // else if(status == OsqpEigen::Status::PrimalInfeasibleInaccurate)
    //     ROS_ERROR("Solver PrimalInfeasibleInaccurate.");

    dq_cmd = dq_solution_first;
    std::cout << "dq cmd :" << dq_cmd.transpose() << std::endl;
}

void Tactile_HQP::publishCommand(){
    for(size_t i=0; i<15; ++i)
    {
        dq_to_pub[i].data = dq_cmd(i);
        dq_pub[i].publish(dq_to_pub[i]);
    }
}

void Tactile_HQP::spin(){
    while (!joint_state_sub.getNumPublishers())
        loop_rate.sleep();

    ROS_INFO_STREAM("Initializing");
    robotInitPose(init_joints_pose);

    ROS_INFO_STREAM("Program starts");
    sleep(1);

    while(ros::ok())
    {
        ros::spinOnce();

        double t1 = ros::Time::now().toSec();

        getCommand();

        markerVisualization();

        publishCommand();
        // calculate average time
        double t2 = ros::Time::now().toSec()-t1;
        sum_time += 1000*t2;
        count += 1;
        cout << "average time : " << sum_time/count << " ms" << endl << endl;
        loop_rate.sleep();
    }
}