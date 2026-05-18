/**
 * @file decision_node_main.cpp
 * @brief 推土机决策节点入口
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 包含模块: 路径规划、主状态机、空间判断、平整度检测、过载检测、决策逻辑
 *
 * 运行方式: rosrun qt_app decision_node
 */
#include <ros/ros.h>
#include "qt_app/log_helper.h"
#include "decision_system.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "decision_node");
    ros::NodeHandle nh;

    initLogSystem();

    LOG_INFO("========================================");
    LOG_INFO("Decision Node started");
    LOG_INFO("========================================");

    try {
        DecisionSystem decision_system(nh);

        LOG_INFO("Decision system running at 100Hz");

        ros::spin();

    } catch (const std::exception& e) {
        LOG_ERROR("Exception in decision_node: %s", e.what());
        destroyLogSystem();
        return 1;
    }

    LOG_INFO("Decision node shutting down...");
    destroyLogSystem();
    return 0;
}
