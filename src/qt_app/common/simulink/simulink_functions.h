/**
 * @file simulink_functions.h
 * @brief Simulink移植算法库 — 总包含头
 * @details 拆分为以下模块:
 *   - simulink_coord.h      坐标变换 (LLA↔ENU, 矩阵运算)
 *   - simulink_moldboard.h  铲刀位姿计算 (杆臂变换)
 *   - simulink_decision.h   决策逻辑/过载/空间判断
 *   - simulink_path.h       旧版路径规划 (保留参考)
 *
 * @author dozer-dev
 * @date 2026-03-15
 */
#ifndef SIMULINK_FUNCTIONS_H
#define SIMULINK_FUNCTIONS_H

#include "simulink_coord.h"
#include "simulink_moldboard.h"
#include "simulink_decision.h"
#include "simulink_path.h"

#endif // SIMULINK_FUNCTIONS_H
