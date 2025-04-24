/**
 * @file init.cpp
 * @brief Program entrance
 * @author Yu Sun,Cong Xiao
 * @maintainer Lipeng Chen
 * @version 1.0.0
 * @date 10.28 2023
 */

#include <tactile_compliance/MainController.hpp>

int main(int argc,char **argv)
{
    ros::init(argc, argv, "tactile_compliance");
    Tactile_HQP tactile;
    tactile.spin();
}