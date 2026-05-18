/**
 * @file ros_to_can_main.cpp
 * @brief Ros To Can Main
 * @author dozer-dev
 * @date 2026-03-15
 */
//
// Created by fzn on 9/14/21.
//
#include "ros_to_can.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_to_can");
    ros::NodeHandle nh;

    ros_to_can n(nh);
    ros::spin();
    return 0;
}
