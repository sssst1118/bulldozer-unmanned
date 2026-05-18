/**
 * @file control_functions.h
 * @brief 控制函数库 — 总包含头
 * @details 拆分为以下模块:
 *   - control_kalman.h       卡尔曼滤波速度融合
 *   - control_td.h           跟踪微分器
 *   - control_pid_utils.h    PID辅助(积分限制/复位/限幅等)
 *   - control_kinematics.h   运动学 + LLA↔ENU + 坐标变换
 *   - control_nonlinear.h    非线性层/死区/阀门死区
 *   - control_moldboard.h    铲刀控制辅助
 *   - control_walk_planning.h 行走/旋转规划 (保留参考, Issue 10后不再被control_node使用)
 *   - control_drive_assist.h  档位/转向/路径跟踪纠偏
 *
 * @author dozer-dev
 * @date 2026-03-15
 */
#ifndef CONTROL_FUNCTIONS_H
#define CONTROL_FUNCTIONS_H

#include "control_kalman.h"
#include "control_td.h"
#include "control_pid_utils.h"
#include "control_kinematics.h"
#include "control_nonlinear.h"
#include "control_moldboard.h"
#include "control_walk_planning.h"
#include "control_drive_assist.h"

#endif // CONTROL_FUNCTIONS_H
