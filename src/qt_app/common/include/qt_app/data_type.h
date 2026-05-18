/**
 * @file data_type.h
 * @brief Data Type
 * @author dozer-dev
 * @date 2026-03-15
 */
#ifndef QT_APP_DATA_TYPE_H
#define QT_APP_DATA_TYPE_H

#include <cstdint>

// 1. 车辆状态：控制节点采集（履带速度、铲刀高度等），发给规划/QT/CAN节点
struct VehicleState {
    double track_left_speed = 0.0;   // 左履带速度 (m/s)
    double track_right_speed = 0.0;  // 右履带速度 (m/s)
    double blade_height = 0.0;       // 铲刀高度 (m)
    bool is_auto_mode = false;       // 自动/手动模式（false=手动，true=自动）
    bool is_emergency = false;       // 紧急停止状态（true=已急停）
    uint64_t timestamp = 0;          // 时间戳（ms），工控机多模块同步必备
};

// 2. 控制指令：QT/规划节点下发，控制节点接收（驱动履带/铲刀）
struct ControlCmd {
    double track_left_target = 0.0;  // 左履带目标速度 (m/s)
    double track_right_target = 0.0; // 右履带目标速度 (m/s)
    double blade_target = 0.0;       // 铲刀目标高度 (m)
    bool emergency_stop = false;     // 紧急停止指令（最高优先级）
    bool switch_auto = false;        // 自动/手动切换指令
    uint64_t timestamp = 0;          // 时间戳（ms）
};

// 3. CAN帧数据：CAN桥接节点专用（ROS和CAN总线之间转发）
struct CanFrame {
    uint32_t id = 0;                 // CAN帧ID（对应推土机的硬件定义）
    uint8_t data[8] = {0};           // CAN数据段（固定8字节）
    uint8_t len = 0;                 // 实际数据长度（1-8）
    bool is_ros2can = false;         // 方向：true=ROS→CAN，false=CAN→ROS
    uint64_t timestamp = 0;          // 时间戳（ms）
};

#endif // QT_APP_DATA_TYPE_H
