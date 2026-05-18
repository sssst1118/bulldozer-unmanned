/**
 * @file can_to_ros_main.cpp
 * @brief Can To Ros Main
 * @author dozer-dev
 * @date 2026-03-15
 */
//
// Created by fzn on 9/14/21.
//
#include "can_to_ros.h"
#include <chrono>


// 主函数
int main(int argc, char** argv)
{
    //ROS
    ros::init(argc, argv, "can_to_ros");
    ros::NodeHandle nh;

    can_to_ros n(nh);
    n.read_can();
    
    return 0;
}