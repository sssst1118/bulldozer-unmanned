# Moldboard Node (铲刀控制节点)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

铲刀控制器负责计算铲刀位姿，通过RTK天线位置+车体姿态+铲刀IMU经杆臂变换得到铲刀端点坐标。

## 计算链路

```
RTK天线位置 (PointA_lla)
    + 车体姿态 (Yaw/Pitch/Roll)
    + 铲刀IMU (Pitch/Roll)
    → 杆臂变换
    → 铲刀右端 (PointD_lla) + 左端 (PointM_lla)
    → 平均高度 (Avg_Height_Actual)
```
