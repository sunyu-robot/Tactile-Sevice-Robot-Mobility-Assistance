/**
 * @file CollisionConstraint.hpp
 * @brief get constraint by CBF
 * @author Yu Sun,Cong Xiao
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
    // Constructor
    CollisionConstraint();

    // update Linear Constraints Matrix
    void updateConstraintsMatrix(const double alpha ,const sphere* point_sphere, Eigen::SparseMatrix<double> &LinearConstraintsMatrix,Matrix<double, 815, 1> &lowerBound,Matrix<double, 815, 1> &upperBound, Vector15d q);

private:
    // distance function for control barrier function 
    std::queue<double> h_cbf;
    // distance vector
    std::queue<Vector3d> distance;
    // cbf vector
    Matrix<double, 815, 1> upper_bound_cbf;
    Matrix<double, 815, 1> lower_bound_cbf;
    // jacobian (constraint matrix) for cbf
    Matrix<double, -1, 15> jacobian_cbf;
    // row of jacobian_cbf
    std::queue<Eigen::Matrix<double, 1, 15>> row_jacobian;
    // size of jacobian
    int size;
    // Identity matrix
    Matrix15d I15d;
    // joint position upper bound & lower bound
    Vector15d joint_limit_ub, joint_limit_lb;
};