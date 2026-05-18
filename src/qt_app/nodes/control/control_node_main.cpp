/**
 * @file control_node_main.cpp
 * @brief 推土机运动控制节点入口
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 包含模块: 卡尔曼滤波速度融合、TD信号平滑、PID速度控制、运动学、铲刀PID控制
 *
 * 运行方式: rosrun qt_app control_node
 */
#include <ros/ros.h>
#include "qt_app/log_helper.h"
#include "control_node.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    initLogSystem();

    LOG_INFO("========================================");
    LOG_INFO("Control Node started");
    LOG_INFO("========================================");

    ControlNode control_node(nh);

    ros::spin();

    LOG_INFO("Control node shutting down...");
    destroyLogSystem();
    return 0;
}
