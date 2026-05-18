# 推土机无人驾驶规控系统

> **Author:** dozer-dev
> **Version:** V1.0

> **声明**：本项目为作者个人的业余爱好项目，全部代码均独立自主编写完成，未使用任何公司内部资料、代码或设备，与作者雇主的任何商业产品或项目无关。本项目仅供学习交流使用，不具备任何商业用途或工程安全性承诺。

## 概述

推土机无人驾驶规控系统，基于 ROS (Noetic) + Qt5 开发，实现推土机自动推土作业的完整控制链路。

> **注意**：`can_to_ros` 与 `ros_to_can` 节点中的 CAN 帧 ID 均为**示例占位值**。实际部署时，需根据目标车辆的 CAN 总线协议文档逐一替换，方可驱动真实车辆。

## 系统架构

```
感知层 (RTK/IMU/栅格地图)
    ↓
can_to_ros_node — CAN数据接收，发布ROS话题
    ↓
decision_node — 路径生成 + 通用路径执行器
    ↓
control_node — PID速度控制 + 铲刀PID控制
    ↓
ros_to_can_node — ROS话题转CAN发送
    ↓
执行层 (履带驱动/液压阀)
```

## 节点列表

| 节点 | 功能 | 频率 |
|------|------|------|
| can_to_ros_node | CAN数据接收，发布ROS话题 | 事件驱动 |
| ros_to_can_node | ROS话题转CAN发送 | 事件驱动 |
| decision_node | 路径规划 + 状态机执行 | 100Hz |
| control_node | 运动控制 (PID) + 铲刀控制 | 100Hz |
| moldboard_node | 铲刀位姿计算 | 50Hz |
| qt_app_node | 监控台界面 (8个Tab) | -- |

## 编译与运行

```bash
# 在 ROS Noetic 工作空间中
cd <workspace>/src
git clone <本仓库>
cd <workspace>
catkin_make
source devel/setup.bash
roslaunch qt_app full_system.launch
```

## 目录结构

```
src/qt_app/
├── common/                 # 公共模块
│   ├── include/qt_app/     # log_helper, data_type, tool_utils
│   ├── simulink/           # 算法函数库 (坐标变换、路径规划等)
│   └── src/                # 公共模块实现
├── nodes/
│   ├── can_to_ros/         # CAN接收节点
│   ├── ros_to_can/         # CAN发送节点
│   ├── decision/           # 决策系统 (路径生成+执行器)
│   ├── control/            # 运动控制节点
│   ├── moldboard/          # 铲刀控制节点
│   └── qt_app_main/        # 监控台界面
├── launch/                 # 启动文件
├── msg/                    # 自定义消息
└── CMakeLists.txt
```

## 监控台界面 (qt_app_node)

| Tab | 功能 |
|-----|------|
| 1.直接CAN | 按钮直驱CAN，绕过规控节点 |
| 2.节点测试 | 自动作业/单步行走/铲刀找平 + 栅格地图 |
| 3.PID调参 | 行走PID + 铲刀PID 在线调节 |
| 4.整车状态 | 发动机/变速箱/电流/姿态 |
| 5.规划参数 | 路径增益/容差/过载阈值 |
| 6.车辆配置 | 参数保存/加载 (.txt) |
| 7.数据录制 | rosbag一键录制 |
| 8.通信诊断 | 话题频率/超时检测 |
