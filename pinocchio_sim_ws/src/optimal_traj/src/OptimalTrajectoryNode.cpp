/**
 * @file OptimalTrajectoryNode.cpp
 * @brief Program entrance
 * @author Yu Sun,Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 04.19 2023
 */

#include <optimal_traj/Trajectory.hpp>
// #include <optimal_traj/DummyHuman.hpp>

int main(int argc, char **argv)
{
    /// Init ros node
    ros::init(argc, argv, "trajectory", ros::init_options::AnonymousName);
    OptimalTrajectory test;
    test.spin();
}