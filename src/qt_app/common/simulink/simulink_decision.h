/**
 * @file simulink_decision.h
 * @brief 决策算法: 综合决策输出、过载检测、空间判断、平整度检测、航向初始化、RTK状态
 */
#ifndef SIMULINK_DECISION_H
#define SIMULINK_DECISION_H

#include <cmath>
#include <array>
#include <cstdint>

namespace simulink {

//--- 航向规划初始化 ---
inline double Heading_Plan_Init(double RTK_Yaw, double Main_Switch, double Main_Switch_) {
    return (Main_Switch_ == 0 && Main_Switch == 1) ? RTK_Yaw : 0.0;
}

//--- RTK状态判断 (当前固定返回1) ---
inline double RTK_Status_Judge(double, double) { return 1.0; }

//--- 空间到位判断 ---
struct SpaceJudgementOutput {
    double Run_X_Spatial_Judge, Run_Theta_Spatial_Judge;
    double Mold_Height_Judge, Mold_Theta_Judge;
};

inline SpaceJudgementOutput Space_Judgement(
    const std::array<double,4>& Error,
    const std::array<double,4>& Tol,
    double P_Counter)
{
    if (P_Counter == 0) return {3,3,3,3};
    auto judge = [](double e, double t) -> double {
        if (e < t)     return 1;
        if (e < t + 5) return 2;
        return 3;
    };
    return { judge(Error[0],Tol[0]), judge(Error[1],Tol[1]),
             judge(Error[2],Tol[2]), judge(Error[3],Tol[3]) };
}

//--- 平整度检测时机 ---
struct FlatnessDetectOutput { double Flatness_Detect_Flag; double Flatness_Detect_Status; };

inline FlatnessDetectOutput Flatness_Detect_Timing(
    double Walking_Completed_Flag, double Direction, double is_flat,
    double, double, double, double,
    double Flatness_Detect_Flag_, double Flatness_Detect_Status_)
{
    FlatnessDetectOutput out;
    if (Walking_Completed_Flag == 2 && Direction == 1) {
        if      (is_flat == 1) out.Flatness_Detect_Status = 1;
        else if (is_flat == 0) out.Flatness_Detect_Status = 2;
        else                   out.Flatness_Detect_Status = 0;
    } else if (Direction == -1) {
        out.Flatness_Detect_Status = 0;
    } else {
        out.Flatness_Detect_Status = Flatness_Detect_Status_;
    }

    if      (Flatness_Detect_Status_ == 0 && out.Flatness_Detect_Status == 1) out.Flatness_Detect_Flag = 1;
    else if (Flatness_Detect_Status_ == 1 && out.Flatness_Detect_Status == 2) out.Flatness_Detect_Flag = 2;
    else if (Flatness_Detect_Status_ == 2 && out.Flatness_Detect_Status == 0) out.Flatness_Detect_Flag = 0;
    else out.Flatness_Detect_Flag = Flatness_Detect_Flag_;
    return out;
}

//--- 综合决策输出 (数组版) ---
struct DecisionOutput {
    double buzz_flag, control_speed_gain, mold_control_plan_flag;
    int16_t Uneven_Flag;
    double Decision_Status;
};

inline DecisionOutput Decision_Output_Calc(
    const double* risk_state, size_t n,
    double csg_risk, double csg_mold,
    double mold_state, double slow_flag,
    double mold_height_d, double mold_limit,
    double Direction, int16_t Uneven_Flag_Plan)
{
    DecisionOutput out = {0, 1, 1, 0, 0};
    double mx = 0;
    for (size_t i = 0; i < n; ++i) if (risk_state[i] > mx) mx = risk_state[i];

    if (mx == 2) {
        out = {1, 0, 0, 0, 1};
    } else {
        out.Uneven_Flag = Uneven_Flag_Plan;
        if (mx == 1) {
            out = {1, csg_risk, 1, Uneven_Flag_Plan, 2};
        } else if (mold_state == 1 && slow_flag == 1 && Direction == 1 && mold_height_d >= mold_limit) {
            out = {0, csg_mold, 1, Uneven_Flag_Plan, 3};
        } else {
            out = {0, 1, 1, Uneven_Flag_Plan, 5};
        }
    }
    return out;
}

//--- 综合决策输出 (4x4矩阵版) ---
struct DecisionControlOutput {
    double buzz_flag, control_speed_gain, mold_control_plan_flag;
    int16_t Uneven_Flag;
    double Decision_Status;
};

inline DecisionControlOutput Decision_Control_Output(
    const std::array<std::array<double,4>,4>& risk_state,
    double csg_risk, double csg_mold,
    double mold_state, double slow_flag,
    double mold_height_d, double mold_limit,
    double Direction, int16_t Uneven_Flag_Plan)
{
    double mx = 0;
    for (auto& r : risk_state) for (auto v : r) if (v > mx) mx = v;

    DecisionControlOutput out = {0, 1, 1, 0, 0};
    if (mx == 2) {
        out = {1, 0, 0, 0, 1};
    } else {
        out.Uneven_Flag = Uneven_Flag_Plan;
        if (mx == 1) {
            out = {1, csg_risk, 1, Uneven_Flag_Plan, 2};
        } else if (mold_state == 1 && slow_flag == 1 && Direction == 1 && mold_height_d >= mold_limit) {
            out = {0, csg_mold, 1, Uneven_Flag_Plan, 3};
        } else {
            out = {0, 1, 1, Uneven_Flag_Plan, 5};
        }
    }
    return out;
}

//--- 推土过载检测 ---
struct OverloadOutput { int16_t OverLoad; double count; };

inline OverloadOutput Earth_Push_Overload(
    double time_num, int16_t Walking_State,
    double Trans_Speed, double Vehicle_Speed, double ASpeed,
    double count_,
    double Trans_Limit, double Vehicle_Limit, double Angular_Limit)
{
    OverloadOutput out = {0, 0};
    if (Walking_State == 2 && Trans_Speed > Trans_Limit &&
        Vehicle_Speed < Vehicle_Limit && std::abs(ASpeed) < Angular_Limit) {
        out.count = count_ + 1;
        if (out.count >= time_num) out.OverLoad = 1;
    }
    return out;
}

} // namespace simulink
#endif // SIMULINK_DECISION_H
