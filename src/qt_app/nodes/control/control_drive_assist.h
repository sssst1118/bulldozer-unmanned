/**
 * @file control_drive_assist.h
 * @brief 行驶辅助: 档位控制、转向调整、路径跟踪纠偏 (Adjust)
 */
#ifndef CONTROL_DRIVE_ASSIST_H
#define CONTROL_DRIVE_ASSIST_H

#include <cmath>

namespace control {

//--- 转向调整 ---
inline double TurnAdjust(double turn_left, double turn_right) {
    return (turn_left == 1 || turn_right == 1) ? 255 : 0;
}

//--- 档位和刹车 ---
struct GearBrakeOutput { double Gear_shifting; double u_brake; };
inline GearBrakeOutput GearBrakeControl(double dir, double BDP, double tf, double csf) {
    if (BDP == 3 && dir > 0 && tf == 0 && csf != 0) return {1, 0};
    if (BDP == 3 && dir < 0 && tf == 0 && csf != 0) return {2, 0};
    return {0, 60};
}

//--- 行走方向选择 ---
inline double WalkDirectionForward(double dir, double fwd, double bwd)  { return (dir > 0) ? fwd : bwd; }
inline double WalkDirectionBackward(double dir, double fwd, double bwd) { return (dir < 0) ? fwd : bwd; }

//--- 旋转档位控制 ---
struct RotationGearOutput { double Gear_shifting; double u_brake; double turn_right; double u_turn; };
inline RotationGearOutput RotationGearControl(double dir, double BDP, double tf, double u_turn_set, double State) {
    if (BDP == 3 && dir > 0 && tf == 0 && State == 1) return {2, 0, 1, u_turn_set};
    if (BDP == 3 && dir < 0 && tf == 0 && State == 1) return {1, 0, 1, u_turn_set};
    return {0, 90, 0, 0};
}

inline double TurnFinal(double u_turn, double u_turn_adjust) {
    return (u_turn == 255 || u_turn_adjust == 255) ? 255 : 0;
}

//--- 速度增益控制 ---
inline double VelocityGainControl(double flag, double v) { return flag * v; }

//--- 发动机转速控制 ---
inline double EngineSpeedControl(double State, double u) { return (State == 6) ? 1600 : u; }

//--- 角速度PID参数选择 ---
struct AngleVelocityPIDOutput { double angle_velocity_kp; double angle_velocity_ki; };
inline AngleVelocityPIDOutput AngleVelocityPIDSelect(double v_ref,
    double kp_f, double ki_f, double kp_b, double ki_b) {
    return (v_ref >= 0) ? AngleVelocityPIDOutput{kp_f, ki_f} : AngleVelocityPIDOutput{kp_b, ki_b};
}

//--- 行走前后分离 ---
struct WalkForwardBackOutput { double u_walk_forward; double u_walk_back; };
inline WalkForwardBackOutput WalkForwardBackSplit(double u) {
    if (u > 0) return {0, u};
    return {std::abs(u), 0};
}

inline WalkForwardBackOutput WalkForwardBackWithAdjust(
    double u_walk,
    double llf, double tlf, double llb, double tlb,
    double lrf, double trf, double lrb, double trb,
    double tr_adj, double tl_adj)
{
    WalkForwardBackOutput out;
    if (u_walk > 0) {
        out.u_walk_back = 0;
        if (tr_adj == 1 && tl_adj == 0)      out.u_walk_forward = u_walk + llf + tlf;
        else if (tr_adj == 0 && tl_adj == 1)  out.u_walk_forward = u_walk + lrf + trf;
        else                                  out.u_walk_forward = u_walk;
    } else {
        out.u_walk_forward = 0;
        if (tr_adj == 1 && tl_adj == 0)      out.u_walk_back = std::abs(u_walk) + llb + tlb;
        else if (tr_adj == 0 && tl_adj == 1)  out.u_walk_back = std::abs(u_walk) + lrb + trb;
        else                                  out.u_walk_back = std::abs(u_walk);
    }
    return out;
}

//--- 旋转运动学 ---
struct RotationKinematicsOutput { double V_right_reference_r; double V_left_reference_r; };
inline RotationKinematicsOutput RotationKinematics(double w) { return {0, w}; }

//--- 路径跟踪调整 (Adjust) ---
struct AdjustOutput {
    double e_theta_left_forward, e_theta_right_forward;
    double e_theta_left_backward, e_theta_right_backward;
    double e_lateral_left_forward, e_lateral_right_forward;
    double e_lateral_left_backward, e_lateral_right_backward;
    double turn_left, turn_right, adjust_flag;
};

inline AdjustOutput Adjust(
    double e_theta, double e_lateral,
    double dz_theta, double dz_lateral,
    double direction, double terminal_flag, double State)
{
    AdjustOutput out = {};

    if (State != 2 && State != 6) { out.adjust_flag = 0; return out; }
    if (terminal_flag == 1) { out.adjust_flag = 1; return out; }

    auto at = std::abs(e_theta), al = std::abs(e_lateral);
    bool tl = (at <= dz_theta), ll = (al <= dz_lateral);

    if (direction > 0) {
        if ((e_theta < 0 && e_lateral < 0) || (e_theta == 0 && e_lateral < 0) || (e_theta < 0 && e_lateral == 0)) {
            if (tl && ll) { out.adjust_flag = 2; }
            else if (tl)  { out.e_lateral_left_forward = al; out.turn_right = 1; out.adjust_flag = 3; }
            else if (ll)  { out.e_theta_left_forward = at;   out.turn_right = 1; out.adjust_flag = 4; }
            else          { out.e_theta_left_forward = at; out.e_lateral_left_forward = al; out.turn_right = 1; out.adjust_flag = 5; }
        } else if ((e_theta > 0 && e_lateral < 0) || (e_theta < 0 && e_lateral > 0) || (e_theta == 0 && e_lateral == 0)) {
            out.adjust_flag = 6;
        } else if ((e_theta > 0 && e_lateral == 0) || (e_theta == 0 && e_lateral > 0) || (e_theta > 0 && e_lateral > 0)) {
            if (tl && ll) { out.adjust_flag = 7; }
            else if (tl)  { out.e_lateral_right_forward = al; out.turn_left = 1; out.adjust_flag = 8; }
            else if (ll)  { out.e_theta_right_forward = at;   out.turn_left = 1; out.adjust_flag = 9; }
            else          { out.e_theta_right_forward = at; out.e_lateral_right_forward = al; out.turn_left = 1; out.adjust_flag = 10; }
        } else { out.adjust_flag = 11; }
    } else {
        if ((e_theta == 0 && e_lateral < 0) || (e_theta > 0 && e_lateral < 0) || (e_theta > 0 && e_lateral == 0)) {
            if (tl && ll) { out.adjust_flag = 12; }
            else if (tl)  { out.e_lateral_left_backward = al; out.turn_right = 1; out.adjust_flag = 13; }
            else if (ll)  { out.e_theta_left_backward = at;   out.turn_right = 1; out.adjust_flag = 14; }
            else          { out.e_theta_left_backward = at; out.e_lateral_left_backward = al; out.turn_right = 1; out.adjust_flag = 15; }
        } else if ((e_theta < 0 && e_lateral < 0) || (e_theta == 0 && e_lateral == 0) || (e_theta > 0 && e_lateral > 0)) {
            out.adjust_flag = 16;
        } else if ((e_theta < 0 && e_lateral == 0) || (e_theta < 0 && e_lateral > 0) || (e_theta == 0 && e_lateral > 0)) {
            if (tl && ll) { out.adjust_flag = 17; }
            else if (tl)  { out.e_lateral_right_backward = al; out.turn_left = 1; out.adjust_flag = 18; }
            else if (ll)  { out.e_theta_right_backward = at;   out.turn_left = 1; out.adjust_flag = 19; }
            else          { out.e_theta_right_backward = at; out.e_lateral_right_backward = al; out.turn_left = 1; out.adjust_flag = 20; }
        } else { out.adjust_flag = 21; }
    }
    return out;
}

} // namespace control
#endif // CONTROL_DRIVE_ASSIST_H
