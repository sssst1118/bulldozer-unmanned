# ROS to CAN Node (CAN发送节点)

> **Author:** dozer-dev  
> **Date:** 2026-03-15

## 概述

ROS话题转CAN总线发送节点，将控制指令发送到车辆执行器。

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| /U_Lever1 | Int16MultiArray[7] | 行走控制 [左转,右转,转向值,升挡,降挡,挡位,解锁] |
| /U_Lever_Moldboard | Int16MultiArray[4] | 铲刀控制 [升,降,左倾,右倾] |
| /U_Brake | Int32 | 刹车 (1=制动) |
| /U_Engine_Speed | Int32 | 发动机转速设定 |
| /Moldboard_Control_Flag_Plan | Int16 | 铲刀控制开关 |
