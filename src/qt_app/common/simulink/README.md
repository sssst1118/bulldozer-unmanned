# Simulink Functions (算法函数库)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

推土机控制算法函数库，包含坐标变换、路径规划、空间判断、决策逻辑等核心算法。

## 主要函数

| 函数 | 功能 |
|------|------|
| lla2enu_internal | LLA→ENU坐标转换 (WGS84椭球模型) |
| Moldboard_Pose_Calc | 铲刀位姿计算 (杆臂变换) |
| Space_Judgement | 空间到位判断 (4维误差评估) |
| Decision_Output_Calc | 综合决策输出 (风险/速度/蜂鸣) |
| Earth_Push_Overload | 推土过载检测 |
| Flatness_Detect_Timing | 平整度检测时机 |

## 常量

| 常量 | 值 | 说明 |
|------|-----|------|
| INIT_ORIGIN | (34.400686, 117.470578, 45.256) | ENU原点 (WGS84) |
| 杆臂值 | 见代码 | RTK天线→铲刀端点标定值 |
