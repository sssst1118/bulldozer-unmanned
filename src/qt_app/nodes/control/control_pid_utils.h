/**
 * @file control_pid_utils.h
 * @brief PID辅助函数: 积分限制、复位、清零、限幅等
 */
#ifndef CONTROL_PID_UTILS_H
#define CONTROL_PID_UTILS_H

#include <cmath>
#include <algorithm>

namespace control {

/// PID积分项限制 - X方向 (误差<0时清正积分, >0时清负积分)
inline double PID_IntegralLimit_X(double u, double Ref_e_X) {
    if (Ref_e_X < 0) return (u > 0) ? 0 : u;
    if (Ref_e_X > 0) return (u < 0) ? 0 : u;
    return u;
}

/// PID积分项限制 - Theta方向 (与X方向逻辑相反)
inline double PID_IntegralLimit_Theta(double u, double Ref_e_Theta) {
    if (Ref_e_Theta > 0) return (u > 0) ? u : 0;
    if (Ref_e_Theta < 0) return (u < 0) ? u : 0;
    return u;
}

/// 复位函数 (main_switch==0 时输出0)
inline double Reset(double u, double main_switch) {
    return (main_switch == 0) ? 0 : u;
}

/// 积分清零 (参考值变化时清零)
inline double ClearIntegral(double u, double Ref, double Ref_prev) {
    return (Ref != Ref_prev) ? 0 : u;
}

/// 带Main_Switch的积分清零
inline double ClearIntegralWithSwitch(double u, double Ref_X, double Ref_X_, double Main_Switch) {
    return (Ref_X != Ref_X_ || Main_Switch == 0) ? 0 : u;
}

/// 角度归一化到 [-180, 180]
inline double normalizeAngle(double angle) {
    while (angle > 180)  angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

/// 限幅函数
inline double saturate(double value, double min_val, double max_val) {
    return std::max(min_val, std::min(max_val, value));
}

/// 限幅 (对称)
inline double Saturation(double u, double u_max) {
    if (u > u_max)  return u_max;
    if (u < -u_max) return -u_max;
    return u;
}

/// 零值保持
inline double ZeroHold(double value, double value_prev) {
    return (value == 0) ? value_prev : value;
}

} // namespace control
#endif // CONTROL_PID_UTILS_H
