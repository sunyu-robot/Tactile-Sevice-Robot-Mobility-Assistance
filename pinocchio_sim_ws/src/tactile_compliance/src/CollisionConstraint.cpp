/*
 * Copyright (c) 2023 Yu Sun <sunyuu@zju.edu.cn>
 * All rights reserved.
 */
#include <tactile_compliance/CollisionConstraint.hpp>

CollisionConstraint::CollisionConstraint()
{
    I15d.setIdentity();
    jacobian_cbf.resize(815, 15);

    // Joint position limits [rad] for the 15-DOF robot (base xyz + left arm + right arm)
    joint_limit_ub <<  180,  180,  180,  3.7,  0.3,  0.3, -1.4708,  3.20,   0.05, -3.1, -2.9,  0.3, -1.4708,  3.2416,  0.1;
    joint_limit_lb << -180, -180, -180,  3.1, -0.2, -0.4, -1.6708,  3.00,  -0.05, -3.7, -3.5, -0.3, -1.6708,  3.0416, -0.1;
}

void CollisionConstraint::updateConstraintsMatrix(
    const double alpha, const sphere* point_sphere,
    Eigen::SparseMatrix<double> &LinearConstraintsMatrix,
    Matrix<double, 815, 1> &lowerBound,
    Matrix<double, 815, 1> &upperBound,
    Vector15d q)
{
    size = 0;
    const int numSpheres = 35;

    // Collect active collision pairs and compute CBF values h(q) = ||p_i - p_j||² - (r_i+r_j)²
    for (int i = 0; i < numSpheres; i++) {
        for (int k = 0; k < (int)point_sphere[i].collision_check.size(); k++) {
            if (point_sphere[i].collision_check[k] != i && size < 770) {
                int j = point_sphere[i].collision_check[k];
                Vector3d d = point_sphere[i].position - point_sphere[j].position;
                distance.push(d);
                h_cbf.push(d.squaredNorm() - pow(point_sphere[i].radius + point_sphere[j].radius, 2));
                row_jacobian.push(2 * d.transpose() * point_sphere[i].jacobian_matrix);
                size++;
            }
        }
    }
    std::cout << "size: " << size << std::endl;

    // Rows 0-29: reserved, unconstrained
    for (int i = 0; i < 30; i++) {
        jacobian_cbf.row(i)  = Eigen::RowVectorXd::Zero(15);
        lower_bound_cbf(i)   = -OsqpEigen::INFTY;
        upper_bound_cbf(i)   =  OsqpEigen::INFTY;
    }

    // Rows 30-(30+size): CBF constraints  J_cbf * dq >= -alpha * h(q)
    for (int i = 30; i < size + 30; i++) {
        jacobian_cbf.row(i)  = row_jacobian.front(); row_jacobian.pop();
        lower_bound_cbf(i)   = -alpha * h_cbf.front(); h_cbf.pop();
        upper_bound_cbf(i)   =  OsqpEigen::INFTY;
    }

    // Rows (30+size)-799: unused CBF slots, unconstrained
    for (int i = size + 30; i < 800; i++) {
        jacobian_cbf.row(i)  = Eigen::RowVectorXd::Zero(15);
        lower_bound_cbf(i)   = -OsqpEigen::INFTY;
        upper_bound_cbf(i)   =  OsqpEigen::INFTY;
    }

    // Rows 800-814: joint position limits enforced via velocity pre-step
    // Clamp to ±0.4 rad/s near limits, otherwise allow full velocity_limit
    jacobian_cbf.bottomRightCorner(15, 15) = Eigen::MatrixXd::Identity(15, 15);
    const double velocity_limit = 2.0;
    const double pre_step       = 0.1;
    for (int i = 800; i < 815; i++) {
        int j = i - 800;
        upper_bound_cbf(i) = ((joint_limit_ub(j) - q(j)) / pre_step < 0.4)
                             ? (joint_limit_ub(j) - q(j)) / pre_step : velocity_limit;
        lower_bound_cbf(i) = ((joint_limit_lb(j) - q(j)) / pre_step > -0.4)
                             ? (joint_limit_lb(j) - q(j)) / pre_step : -velocity_limit;
    }

    LinearConstraintsMatrix = jacobian_cbf.sparseView();
    lowerBound = lower_bound_cbf;
    upperBound = upper_bound_cbf;
}
