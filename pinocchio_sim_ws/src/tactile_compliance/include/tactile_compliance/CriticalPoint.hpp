/**
 * @file CriticalPoint.hpp
 * @brief Collision sphere management for the dual-arm robot
 * @author Yu Sun, Cong Xiao
 * @maintainer Lipeng Chen
 * @version 0.1.0
 * @date 10.28 2023
 */
#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include "Utils.hpp"
#include <visualization_msgs/MarkerArray.h>

using namespace BodyMath;

class CriticalPoint
{
public:
    CriticalPoint() = default;

    /**
     * @brief Assign radii and collision-check pairs to all 35 spheres.
     *
     * Spheres 0-13: right arm links.
     * Spheres 14-27: left arm links.
     * Spheres 28-34: base / torso.
     * Adjacent spheres on the same chain are excluded from mutual checking
     * to avoid false positives at joints.
     */
    void initsphere(sphere (&point_sphere)[35]);

    /**
     * @brief Recompute sphere positions and Jacobians from current joint angles.
     * @param q_real  Full 15-DOF joint position vector.
     */
    void updatedata(const Vector15d q_real, sphere (&point_sphere)[35]);

    // Populate marker_array with green semi-transparent spheres for RViz.
    void pointMarker(visualization_msgs::MarkerArray &marker_array, const sphere* point_sphere);

private:
    visualization_msgs::Marker sphere_marker[35];
};
