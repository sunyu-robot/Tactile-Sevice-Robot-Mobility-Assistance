/*
 * Copyright (c) 2023 Yu Sun <sunyuu@zju.edu.cn>
 * All rights reserved.
 */
#include <tactile_compliance/CollisionConstraint.hpp>

CollisionConstraint::CollisionConstraint()
{
    I15d.setIdentity();
    jacobian_cbf.resize(815,15);

    joint_limit_ub << 180,180,180,3.7, 0.3, 0.3, -1.4708, 3.20, 0.05, -3.1, -2.9, 0.3, -1.4708, 3.2416, 0.1;
    joint_limit_lb << -180,-180,-180,3.1, -0.2, -0.4, -1.6708, 3.00, -0.05, -3.7, -3.5, -0.3, -1.6708, 3.0416, -0.1;
}

void CollisionConstraint::updateConstraintsMatrix(const double alpha ,const sphere* point_sphere, Eigen::SparseMatrix<double> &LinearConstraintsMatrix, Matrix<double, 815, 1> &lowerBound, Matrix<double, 815, 1> &upperBound, Vector15d q)
{
    Vector3d distance_vector;
    size = 0;
    const int numSpheres = 1;
    for (int i = 0; i < numSpheres; i++) {
            for(int k = 0; k < point_sphere[i].collision_check.size(); k++)
            {   
                if(point_sphere[i].collision_check[k] != i)
                {
                    int j = point_sphere[i].collision_check[k];
                    distance_vector = point_sphere[i].position - point_sphere[j].position;
                    // size++;
                    distance.push(distance_vector);
                    h_cbf.push(distance_vector.squaredNorm() - pow((point_sphere[i].radius + point_sphere[j].radius),2));
                    row_jacobian.push(2*distance_vector.transpose()*point_sphere[i].jacobian_matrix); 
                }
            }
        }
    std::cout << "size :" << size << std::endl;

    for(int i = 0;i < 30; i++)
    {
        jacobian_cbf.row(i) = Eigen::RowVectorXd::Zero(15);
        lower_bound_cbf(i) = - OsqpEigen::INFTY;
        upper_bound_cbf(i) = OsqpEigen::INFTY;
    }

    for(int i = 30;i < size + 30; i++)
    {
        jacobian_cbf.row(i) = row_jacobian.front();
        lower_bound_cbf(i) = - alpha * h_cbf.front();
        upper_bound_cbf(i) = OsqpEigen::INFTY;
        row_jacobian.pop();
        h_cbf.pop();
    }
    for(int i = size + 30;i < 800; i++)
    {
        jacobian_cbf.row(i) = Eigen::RowVectorXd::Zero(15);
        lower_bound_cbf(i) = - OsqpEigen::INFTY;
        upper_bound_cbf(i) = OsqpEigen::INFTY;
    }

    jacobian_cbf.bottomRightCorner(15, 15) = Eigen::MatrixXd::Identity(15, 15);

    double velocity_limit = 2;
    double pre_step = 0.1;
    for(int i = 800;i < 815; i++)
    {
        if((joint_limit_ub(i-800) - q(i-800))/pre_step < 0.4)
            upper_bound_cbf(i) = (joint_limit_ub(i-800) - q(i-800))/pre_step;
        else
            upper_bound_cbf(i) = velocity_limit;

        if((joint_limit_lb(i-800) - q(i-800))/pre_step > -0.4)
            lower_bound_cbf(i) = (joint_limit_lb(i-800) - q(i-800))/pre_step;
        else
            lower_bound_cbf(i) = -velocity_limit;
    }

    // constraint matrix
    LinearConstraintsMatrix = jacobian_cbf.sparseView();
    lowerBound = lower_bound_cbf;
    upperBound = upper_bound_cbf;
}