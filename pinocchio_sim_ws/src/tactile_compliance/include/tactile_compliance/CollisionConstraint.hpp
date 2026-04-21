/**
 * @file CollisionConstraint.hpp
 * @brief CBF-based collision avoidance constraints for OSQP
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 0.1.0
 * @date 10.28 2023
 */
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <iostream>
#include "Utils.hpp"
#include "OsqpEigen/OsqpEigen.h"
#include <queue>

using namespace BodyMath;

class CollisionConstraint
{
public:
    CollisionConstraint();

    /**
     * @brief Build the full 815-row constraint matrix for OSQP.
     *
     * Rows 0-29:       reserved (zeroed, unconstrained).
     * Rows 30-(30+n):  CBF constraints: J_cbf * dq >= -alpha * h(q).
     *                  h(q) = ||p_i - p_j||² - (r_i + r_j)² for each sphere pair.
     * Rows 800-814:    joint position limit constraints via velocity pre-step.
     *
     * @param alpha              CBF decay rate (higher = more aggressive avoidance).
     * @param point_sphere       Array of 35 collision spheres with current positions/Jacobians.
     * @param LinearConstraintsMatrix  Output sparse constraint matrix (815×15).
     * @param lowerBound         Output lower bound vector (815×1).
     * @param upperBound         Output upper bound vector (815×1).
     * @param q                  Current joint position vector (15×1).
     */
    void updateConstraintsMatrix(const double alpha, const sphere* point_sphere,
                                 Eigen::SparseMatrix<double> &LinearConstraintsMatrix,
                                 Matrix<double, 815, 1> &lowerBound,
                                 Matrix<double, 815, 1> &upperBound,
                                 Vector15d q);

private:
    std::queue<double>                       h_cbf;        // CBF values h(q) per active pair
    std::queue<Vector3d>                     distance;     // distance vectors per active pair
    std::queue<Eigen::Matrix<double, 1, 15>> row_jacobian; // CBF Jacobian rows
    Matrix<double, 815, 1>  upper_bound_cbf;
    Matrix<double, 815, 1>  lower_bound_cbf;
    Matrix<double, -1, 15>  jacobian_cbf;   // dense constraint matrix before sparse conversion
    int      size;                           // number of active collision pairs this cycle
    Matrix15d I15d;
    Vector15d joint_limit_ub, joint_limit_lb;
};
