/**
 * @file control_moldboard.h
 * @brief 铲刀控制辅助: 中心高度、P控制、I参数选择、积分衰减、输出分离、控制标志
 */
#ifndef CONTROL_MOLDBOARD_H
#define CONTROL_MOLDBOARD_H

#include <cmath>

namespace control {

/// 铲刀中心高度 = 左右平均
inline double MoldboardMiddleHeight(double left, double right) {
    return (left + right) / 2.0;
}

/// 高度P控制 (不对称增益)
inline double HeightPControl(double e, double Kp_Up, double Kp_Down) {
    return (e > 0) ? (e * Kp_Up) : (e * Kp_Down);
}

/// 角度P控制 (不对称增益)
inline double ThetaPControl(double e, double Kp_Up, double Kp_Down) {
    return (e > 0) ? (e * Kp_Up) : (e * Kp_Down);
}

/// 速度I参数选择 (根据误差方向)
inline double VelocityISelect(double e, double I_Left, double I_Right) {
    return (e > 0) ? I_Left : I_Right;
}

/// 积分项角度衰减
inline double IntegralAngleDecay(double u, double e_Angle, double I_angle) {
    if (std::abs(e_Angle) < I_angle && e_Angle != 0)
        return std::sqrt(std::abs(e_Angle) / I_angle) * u;
    return u;
}

/// 铲刀控制标志
inline double MoldboardControlFlag(double flag, double main_switch) {
    return (flag == 1 && main_switch == 1) ? 1 : 0;
}

/// 高度输出分离 (Up/Dn)
struct HeightUpDnOutput { double u_Height_Up; double u_Height_Dn; };
inline HeightUpDnOutput HeightUpDnSplit(double u) {
    if (u > 0) return {u, 0};
    return {0, std::abs(u)};
}

/// 角度输出分离 (Up/Dn)
struct ThetaUpDnOutput { double u_Theta_Up; double u_Theta_Dn; };
inline ThetaUpDnOutput ThetaUpDnSplit(double u) {
    if (u > 0) return {u, 0};
    return {0, std::abs(u)};
}

} // namespace control
#endif // CONTROL_MOLDBOARD_H
