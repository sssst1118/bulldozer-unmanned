/**
 * @file moldboard_node_main.cpp
 * @brief 铲刀位姿计算节点入口
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 包含模块: RTK杆臂变换、铲刀端点坐标计算、高度/角度计算
 *
 * 运行方式: rosrun qt_app moldboard_node
 */
#include <ros/ros.h>
#include "qt_app/log_helper.h"
#include "moldboard_controller.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "moldboard_node");
    ros::NodeHandle nh;

    initLogSystem();

    LOG_INFO("========================================");
    LOG_INFO("Moldboard Controller Node started");
    LOG_INFO("========================================");

    MoldboardController controller(nh);

    ros::spin();

    LOG_INFO("Moldboard node shutting down...");
    destroyLogSystem();
    return 0;
}
