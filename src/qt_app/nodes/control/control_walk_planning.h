/**
 * @file control_walk_planning.h
 * @brief 行走/旋转规划 (Simulink移植)
 * @note [Issue 10] control_node 不再使用此模块做到位判断.
 *       decision_node 是到位判断的唯一权威.
 *       此文件保留供参考和测试.
 */
#ifndef CONTROL_WALK_PLANNING_H
#define CONTROL_WALK_PLANNING_H

#include <cmath>

namespace control {

struct WalkPlanningOutput {
    double v_x;
    double v_theta;
    double terminal_flag;
    double X_terminal;
};

inline WalkPlanningOutput WalkPlanning(
    double state, double v,
    double X_terminal, double Y_terminal, double Theta_terminal,
    double X_Real, double Y_Real, double Theta_Real,
    double X_Tolerance, double Y_Tolerance, double Theta_Tolerance,
    double hold_x, double terminal_flag_, double X_terminal_)
{
    WalkPlanningOutput output;
    output.X_terminal = X_terminal;

    if (state == 2 || state == 6) {
        if (terminal_flag_ == 1) {
            output.v_theta = 0; output.v_x = 0; output.terminal_flag = 1;
            if (X_terminal != X_terminal_) output.terminal_flag = 0;
        } else {
            if (std::abs(X_Real - X_terminal) < X_Tolerance) {
                output.terminal_flag = 1; output.v_theta = 0; output.v_x = 0;
            } else {
                if (v == 0) { output.v_theta = 0; output.v_x = 0; }
                else {
                    if (std::abs(X_terminal - X_Real) > hold_x) {
                        output.v_x = ((X_terminal - X_Real) > 0) ? v : -v;
                        output.v_theta = Theta_terminal - Theta_Real;
                    } else {
                        output.v_theta = Theta_terminal - Theta_Real;
                        output.v_x = (X_terminal - X_Real) * v / hold_x;
                    }
                }
                output.terminal_flag = 0;
            }
        }
    } else if (state == 3) {
        output.terminal_flag = 1; output.v_x = 0; output.v_theta = 0;
    } else {
        output.terminal_flag = 0; output.v_x = 0; output.v_theta = 0;
    }
    return output;
}

struct RotationPlanningOutput {
    double v_theta;
    double terminal_flag_rotation;
    double Theta_terminal;
};

inline RotationPlanningOutput RotationPlanning(
    double state, double v,
    double X_terminal, double Y_terminal, double Theta_terminal,
    double X_Real, double Y_Real, double Theta_Real,
    double X_Tolerance, double Y_Tolerance, double Theta_Tolerance,
    double hold_theta, double terminal_flag_rotation_, double Theta_terminal_)
{
    RotationPlanningOutput output;
    output.Theta_terminal = Theta_terminal;

    if (state == 1) {
        if (terminal_flag_rotation_ == 1) {
            output.v_theta = 0; output.terminal_flag_rotation = 1;
            if (Theta_terminal != Theta_terminal_) output.terminal_flag_rotation = 0;
        } else {
            if (std::abs(Theta_terminal - Theta_Real) < Theta_Tolerance) {
                output.terminal_flag_rotation = 1; output.v_theta = 0;
            } else {
                if (v == 0) { output.v_theta = 0; }
                else {
                    if (std::abs(Theta_terminal - Theta_Real) > hold_theta)
                        output.v_theta = ((Theta_terminal - Theta_Real) > 0) ? v : -v;
                    else
                        output.v_theta = (Theta_terminal - Theta_Real) * v / hold_theta;
                }
                output.terminal_flag_rotation = 0;
            }
        }
    } else if (state == 3 || state == 2) {
        output.terminal_flag_rotation = 1; output.v_theta = 0;
    } else {
        output.terminal_flag_rotation = 0; output.v_theta = 0;
    }
    return output;
}

struct InitialPositionOutput { double x_0; double y_0; double theta_0; };

inline InitialPositionOutput InitialPositionUpdate(
    double x_0_, double y_0_, double theta_0_,
    double theta, double x, double y, double terminal_flag)
{
    if (terminal_flag == 1) return {x, y, theta};
    return {x_0_, y_0_, theta_0_};
}

} // namespace control
#endif // CONTROL_WALK_PLANNING_H
