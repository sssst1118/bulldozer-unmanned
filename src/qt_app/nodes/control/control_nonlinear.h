/**
 * @file control_nonlinear.h
 * @brief 非线性层: 插值表、死区、阀门死区
 */
#ifndef CONTROL_NONLINEAR_H
#define CONTROL_NONLINEAR_H

#include <cmath>

namespace control {

/// 线性插值
inline double interp1(const double* X, const double* Y, int n, double x) {
    if (x <= X[0])   return Y[0];
    if (x >= X[n-1]) return Y[n-1];
    for (int i = 0; i < n - 1; i++) {
        if (x >= X[i] && x <= X[i+1]) {
            double t = (x - X[i]) / (X[i+1] - X[i]);
            return Y[i] + t * (Y[i+1] - Y[i]);
        }
    }
    return Y[n-1];
}

/// 高度非线性层
inline double HeightNonlinearLayer(double u, double P1, double P2, double P3) {
    const double X[] = {-50,-30,-15,-8,-3,0,3,8,15,30,50};
    double Y[] = {-1000,-P3,-P2,-P1,-100,0,100,P1,P2,P3,1000};
    return interp1(X, Y, 11, u);
}

/// 角度非线性层 (与高度相同结构)
inline double ThetaNonlinearLayer(double u, double P1, double P2, double P3) {
    return HeightNonlinearLayer(u, P1, P2, P3);
}

/// 行走非线性层 (terminal_flag==1时输出0)
inline double WalkNonlinearLayer(double u, double P1, double P2, double P3, double terminal_flag) {
    if (terminal_flag == 1) return 0;
    const double X[] = {-50,-30,-15,-8,-3,0,3,8,15,30,50};
    double Y[] = {-1000,-P3,-P2,-P1,-350,0,350,P1,P2,P3,1000};
    return interp1(X, Y, 11, u);
}

/// 行走非线性层 (519变体, 无terminal_flag)
inline double WalkNonlinearLayer519(double u, double P1, double P2, double P3) {
    const double X[] = {-100,-60,-30,-15,-7,0,7,15,30,60,100};
    double Y[] = {-750,-P3,-P2,-P1,-350,0,350,P1,P2,P3,750};
    return interp1(X, Y, 11, u);
}

/// 行走非线性层 (529变体)
inline double WalkNonlinearLayer529(double u, double P1, double P2, double P3, double terminal_flag) {
    if (terminal_flag == 1) return 0;
    const double X[] = {-50,-30,-15,-8,-3,0,3,8,15,30,50};
    double Y[] = {-2000,-P3,-P2,-P1,-750,0,750,P1,P2,P3,2000};
    return interp1(X, Y, 11, u);
}

/// 死区处理
inline double Deadzone(double u, double dz) {
    return (std::abs(u) < dz) ? 0 : u;
}

/// 阀门死区 (高度)
inline double ValveDeadzone(double u, double dz, double devalue) {
    if (u > dz)  return devalue;
    if (u < -dz) return -devalue;
    return 0;
}

/// 阀门死区 (角度)
inline double ThetaValveDeadzone(double u, double dz, double devalue) {
    return ValveDeadzone(u, dz, devalue);
}

} // namespace control
#endif // CONTROL_NONLINEAR_H
