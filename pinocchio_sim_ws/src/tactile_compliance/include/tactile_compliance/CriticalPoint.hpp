/**
 * @file CriticalPoint.hpp
 * @brief get critical point and publish marker
 * @author Yu Sun,Cong Xiao
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
    // Constructor
    CriticalPoint() = default;

    /**
    * @brief init feature of each critical points
    * @detail
    *  init position and jacobian matrix of each critical points
    * @param[in] &input        joints' position\
    * @param[in] &input        struct of sphere
    */
    void initsphere(sphere (&point_sphere)[35]);

    /**
    * @brief update feature of each critical points
    * @detail
    *  update position and jacobian matrix of each critical points
    * @param[in] &input        joints' position
    */
    void updatedata(const Vector15d q, sphere (&point_sphere)[35]);

    void pointMarker(visualization_msgs::MarkerArray &marker_array, const sphere* point_sphere);

private:
    // marker to publish
    visualization_msgs::Marker sphere_marker[35];
    
};