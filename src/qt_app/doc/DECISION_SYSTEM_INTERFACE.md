# 推土机决策系统 ROS 话题接口说明

> **Author:** dozer-dev  
> **Date:** 2026-03-15


本文档描述了C++/ROS的决策系统。

---

## 1. 系统模块组成

| 模块名称 | 功能描述 |
|---------|-------------|
| 路径规划 | 栅格地图路径规划 |
| 主状态机 | 行走决策状态机 |
| RTK状态判断 | RTK定位状态检查 |
| 检测平整度时机 | 平整度检测触发 |
| 决策逻辑 | 风险决策与速度控制 |
| 铲刀中心高度 | 铲刀平均高度计算 |
| 铲刀姿态计算 | 铲刀端点坐标计算 |
| 推土过载检测 | 推土过载判断 |
| 空间判断 | 位置/姿态误差判断 |

---

## 2. 订阅话题 (Inputs)

### 2.1 主控制
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/Main_Switch` | std_msgs/Float64 | 主开关 (0=关, 1=开) |
| `/Detection_Altitude_Completed` | std_msgs/Float64 | 高度检测完成标志 |

### 2.2 定位与姿态
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/RTK` | sensor_msgs/NavSatFix | RTK定位 (纬经高) |
| `/LLA` | geometry_msgs/Point | 纬经高坐标 |
| `/Angle_Heading` | geometry_msgs/Point | 航向角 (yaw, pitch, roll) |
| `/AHRS_IMU` | geometry_msgs/Point | IMU姿态角 |

### 2.3 地图与导航
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/Occupancy_grid` | nav_msgs/OccupancyGrid | 栅格地图 |
| `/OccupancyGrid_location` | geometry_msgs/Point | 栅格定位 (row, col) |
| `/Navigation_Coordinate` | std_msgs/Float64 | 导航坐标角度 |
| `/Walking_Coordinate_Initial` | std_msgs/Float64 | 初始行走坐标 |
| `/Navigate_location` | geometry_msgs/Point | 导航位置 (Run_X, Run_Y, Run_Theta) |
| `/Location_real` | geometry_msgs/Point | 实时位置 |

### 2.4 铲刀相关
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/Avg_Height` | std_msgs/Float64 | 规划平均高度 |
| `/Ref_Mold_Theta` | std_msgs/Float64 | 规划铲刀角度 |
| `/Bulldozer_Moldboard_Left_Dvalue_Plan` | std_msgs/Float64 | 左铲刀位移 |
| `/Bulldozer_Moldboard_Right_Dvalue_Plan` | std_msgs/Float64 | 右铲刀位移 |
| `/Point_Actual_Height` | geometry_msgs/Point | 实际高度点 (left, right) |

### 2.5 风险与状态
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/risk_state` | std_msgs/Float64MultiArray | 风险状态数组 [4] |
| `/Mold_OverLoad_Status` | std_msgs/Int16 | 铲刀过载状态 |

### 2.6 参数配置 (RLS话题)
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/RLS2` | geometry_msgs/Point | 误差容限 (Run_X, Run_Theta, Mold_Height) |
| `/RLS3` | geometry_msgs/Point | is_flat, map_ready |
| `/RLS4` | geometry_msgs/Point | X_Gain, Theta_Gain |
| `/RLS5` | geometry_msgs/Point | Walk_Lock, Moldboard_Control_Lock |
| `/RLS6` | geometry_msgs/Point | 速度增益 (risk, mold, mold_limit) |
| `/RLS8` | geometry_msgs/Point | height_delta, up_height_delta, x_back_set |
| `/RLS9` | geometry_msgs/Point | Uneven_Flag |
| `/RLS10` | geometry_msgs/Point | 过载阈值 (Trans_Speed, Vehicle_Speed, Angular_Speed) |
| `/RLS11` | geometry_msgs/Point | time_num (过载计时阈值) |
| `/W2` | std_msgs/Float64 | 铲刀宽度 |

---

## 3. 发布话题 (Outputs)

### 3.1 状态机输出
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/decision/walk_state` | std_msgs/Float64 | 行走状态 (0-8) |
| `/decision/ref_position` | geometry_msgs/Point | 参考位置 (x, y, theta) |
| `/decision/walking_completed_flag` | std_msgs/Float64 | 行走完成标志 |
| `/decision/moldboard_control_flag` | std_msgs/Float64 | 铲刀控制标志 |
| `/decision/moldboard_complete_flag` | std_msgs/Float64 | 铲刀完成标志 |
| `/decision/action` | std_msgs/Float64 | 动作指令 |
| `/decision/current_p_counter` | std_msgs/Float64 | 当前路径点计数 |
| `/decision/back_counter` | std_msgs/Float64 | 后退计数 |
| `/decision/slow_flag` | std_msgs/Float64 | 减速标志 |
| `/decision/mold_state` | std_msgs/Float64 | 铲刀状态 |
| `/decision/avg_height_plan` | std_msgs/Float64 | 规划平均高度 |
| `/decision/ref_mold_theta_plan` | std_msgs/Float64 | 规划铲刀角度 |

### 3.2 路径规划输出
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/Planning_Complete_Flag` | std_msgs/Float64 | 规划完成标志 |
| `/Path` | geometry_msgs/Point | 路径 (delta_theta, distance) |
| `/Direction` | std_msgs/Float64 | 方向 (-1/0/1) |
| `/Condition_Array` | std_msgs/Float64MultiArray | 规划条件数组 [8] |

### 3.3 决策逻辑输出
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/buzz_flag` | std_msgs/Float64 | 蜂鸣标志 |
| `/control_speed_gain` | std_msgs/Float64 | 速度控制增益 |
| `/Decision_Status` | std_msgs/Float64 | 决策状态 (0-5) |
| `/Uneven_Flag` | std_msgs/Int16 | 不平标志 |

### 3.4 其他输出
| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/Flatness_Detect_Flag` | std_msgs/Float64 | 平整度检测标志 |
| `/Mold_OverLoad_Status` | std_msgs/Int16 | 过载状态 |

---

## 4. 行走状态 (walk_state) 说明

| 值 | 状态名称 | 对应Simulink SSID |
|---|---------|------------------|
| 0 | STOPPED | stop (SSID=51) |
| 1 | ROTATING | rotating (SSID=3) |
| 2 | MOVING | moving_1 (SSID=4) / Back (SSID=191) |
| 3 | ARRIVAL1 | arrival_flag1 (SSID=42) |
| 4 | ARRIVAL2 | arrival_flag2 (SSID=36) |
| 5 | BACK_STOP | Stop2Back (SSID=187) |
| 6 | MOLD_UP | Mold_Up (SSID=223) |
| 8 | UPDATE | Update (SSID=213) |

---

## 5. 决策状态 (Decision_Status) 说明

| 值 | 状态 | 动作 |
|---|------|------|
| 0 | 初始 | 无特殊动作 |
| 1 | 刹车 | max_risk==2: 停止+蜂鸣 |
| 2 | 避障 | max_risk==1: 减速+蜂鸣 |
| 3 | 铲刀 | 动铲刀时减速 |
| 5 | 正常 | 正常运行 |

---

## 6. V3_15_5_1 关键修改

1. **rotating状态**: `theta_ref = theta_terminal` (之前可能使用其他值)
2. **移除wait_rotating**: rotating直接转换到moving_moldboard
3. **Uneven_Flag条件**: 从`==2`改为`==1`

---

## 7. 使用示例

```bash
# 启动决策节点
rosrun qt_app decision_node

# 开启主开关
rostopic pub /Main_Switch std_msgs/Float64 "data: 1.0"

# 查看行走状态
rostopic echo /decision/walk_state

# 查看规划状态
rostopic echo /Planning_Complete_Flag
```

---

## 8. 索引转换注意

MATLAB数组从1开始，C++从0开始：
- `Spatial_Decision(1)` → `Spatial_Decision[0]`
- `Spatial_Decision(2)` → `Spatial_Decision[1]`
- `Condition_Array(1)` → `Condition_Array[0]`
