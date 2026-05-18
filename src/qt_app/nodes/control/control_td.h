/**
 * @file control_td.h
 * @brief 跟踪微分器 (Tracking Differentiator)
 */
#ifndef CONTROL_TD_H
#define CONTROL_TD_H

#include <cmath>

namespace control {

inline double TD(double x1, double x2, double r, double h) {
    double d = r * h * h;
    double y = x1 + h * x2;
    double a1 = std::sqrt(d * (d + 8 * std::abs(y)));

    double a;
    if (std::abs(y) > d) {
        a = h * x2 + (a1 - d) * (y > 0 ? 1 : -1) / 2;
    } else {
        a = h * x2 + y;
    }

    if (std::abs(a) > d) {
        return -r * (a > 0 ? 1 : -1);
    } else {
        return -r * a / d;
    }
}

} // namespace control
#endif // CONTROL_TD_H
