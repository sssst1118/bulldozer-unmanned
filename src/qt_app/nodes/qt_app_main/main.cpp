/**
 * @file main.cpp
 * @brief 推土机手动控制测试节点
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 启动 Qt 界面，提供手动行走和铲刀控制按钮，
 * 用于验证整车 CAN 控制通路是否正常工作。
 *
 * 使用方法:
 *   rosrun qt_app qt_app_node
 *   或通过 roslaunch qt_app full_system.launch 自动启动
 */
#include <QApplication>
#include <QTimer>
#include <ros/ros.h>
#include "qt_app/log_helper.h"
#include "manual_control_widget.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "qt_app_node");
    ros::NodeHandle nh;

    initLogSystem();

    LOG_INFO("========================================");
    LOG_INFO("  Manual Control Test Node");
    LOG_INFO("========================================");

    QApplication app(argc, argv);

    // ROS关闭时自动退出Qt事件循环 (每100ms检查)
    QTimer rosCheckTimer;
    QObject::connect(&rosCheckTimer, &QTimer::timeout, [&]() {
        if (!ros::ok()) QApplication::quit();
    });
    rosCheckTimer.start(100);

    ManualControlWidget widget(nh);
    widget.show();

    // Qt 事件循环, 内部 onTimerTick 会调用 ros::spinOnce()
    int ret = app.exec();

    ros::shutdown();  // 快速断开所有ROS连接
    LOG_INFO("Manual control node shutting down...");
    destroyLogSystem();
    return ret;
}
