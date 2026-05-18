# Control Node (运动控制节点)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

运动控制器负责履带速度控制和铲刀液压控制，运行频率100Hz。

## 控制链路

```
decision_node 输入
    ↓
规划层: WalkPlanning / RotationPlanning
    ↓
运动学层: 差速转向 → V_right_ref / V_left_ref
    ↓
TD滤波: 跟踪微分器平滑
    ↓
PID控制: 跟踪参考速度
    ↓
V_right_cmd / V_left_cmd → 桥接 → /U_Lever1 → CAN
```

## PID 参数 (4套)

| PID | 控制目标 | 参数 |
|-----|----------|------|
| 右履带 | V_right跟踪 | Kp_x, Ki_x, Kd_x |
| 左履带 | V_left跟踪 | Kp_theta, Ki_theta, Kd_theta |
| 铲刀高度 | 目标高度跟踪 | Kp_Height_Up/Down, Ki_Height_Up/Down |
| 铲刀角度 | 目标角度跟踪 | Kp_Theta_Up/Down, Ki_Theta_Up/Down |

## 话题接口

### 订阅
| 话题 | 类型 | 来源 |
|------|------|------|
| /decision/walk_state | Float64 | decision_node |
| /decision/main_switch | Float64 | decision_node |
| /control/terminal | Float64MultiArray[3] | decision_node |
| /control/reference | Float64MultiArray[2] | decision_node |
| /control/params | Float64MultiArray[17] | 界面/配置 |
| /control/moldboard_params | Float64MultiArray[22] | 界面/配置 |

### 发布
| 话题 | 类型 | 说明 |
|------|------|------|
| /U_Lever1 | Int16MultiArray[7] | 行走CAN指令 |
| /U_Lever_Moldboard | Int16MultiArray[4] | 铲刀CAN指令 |
| /control/V_right | Float64 | 右履带速度 |
| /control/V_left | Float64 | 左履带速度 |
| /control/terminal_flag | Float64 | 到位标志 |
