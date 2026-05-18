# CAN to ROS Node (CAN接收节点)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

CAN总线数据接收节点，解析CAN报文并发布为ROS话题。

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /LLA | PointStamped | RTK位置 (x=经度,y=纬度,z=高程) |
| /Angle_Heading | PointStamped | 车体姿态 (x=航向,y=俯仰,z=横滚) rad |
| /AHRS_IMU | sensor_msgs/Imu | 铲刀IMU (roll/pitch在orientation_covariance[0/1]) |
| /RTK | NavSatFix | RTK (lat/lon/alt) |
| /Engine_Speed_Actual | Float64 | 发动机转速 |
| /Engine_Torque_Actual | Int8 | 发动机扭矩% |
| /Transmission_Speed_Actual | Int16 | 变速箱转速 |
| /Vehicle_Speed_Vel | Float64 | 车速 |
| /OUTPUT_CURRENT | UInt16MultiArray[4] | 输出电流 |
