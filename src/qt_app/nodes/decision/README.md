# Decision Node (决策节点)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

决策系统是推土机无人驾驶的核心模块，采用"路径生成 + 通用执行器"架构。

## 架构

```
路径生成器 (path_generator.h)
  ├── Z字推 (ZIGZAG): 推→提刀倒回→换列→推
  └── 单向推 (UNIDIRECTIONAL): 推→提刀回原点→换列→推
        ↓
  vector<WayPoint> 路径点队列
        ↓
通用路径执行器 (decision_system.cpp)
  IDLE → ROTATING → DRIVING → WAYPOINT_DONE → FINISHED
        ↓
  control_node (walk_state + terminal + reference)
```

## 文件说明

| 文件 | 说明 |
|------|------|
| path_types.h | WayPoint、BladeCmd、ExecState 等数据结构 |
| path_generator.h | 路径生成算法 (Z字推/单向推) |
| decision_system.h | 决策系统类定义 |
| decision_system.cpp | 执行器 + 辅助模块实现 |
| decision_node_main.cpp | 节点入口 |

## 顶层状态

| 状态 | 条件 |
|------|------|
| REMOTE_TAKEOVER | 默认安全态，不输出指令 |
| AUTO_OPERATION | Main_Switch=1 且 Detection_Completed=1 |

## 路径点结构

```cpp
struct WayPoint {
    double x, y;         // ENU坐标
    BladeCmd blade_cmd;  // RAISE/LEVEL/HOLD
    DriveDir drive_dir;  // FORWARD/BACKWARD
};
```

## 辅助模块

- 空间判断: 评估到位精度
- 决策逻辑: 风险/速度增益/蜂鸣
- 过载检测: 堵转工况检测
- 铲刀高度计算: RTK杆臂变换
