/**
 * @file simulink_path.h
 * @brief 旧版路径规划算法 (Z字型固定路径, 已弃用)
 * @note 新架构使用 decision_system 中的路径生成器, 此文件保留供参考.
 */
#ifndef SIMULINK_PATH_H
#define SIMULINK_PATH_H

#include <cmath>
#include <array>

namespace simulink {

struct PathPlanningOutput {
    std::array<double,3> Condition_Array;
    double Planning_Complete_Flag, plan_status, Avg_Height_Plan, is_boundary;
    std::array<double,2> Path;
    double direction, walk_num, loop;
};

inline PathPlanningOutput Path_Planning(
    double Main_Switch, double Heading_Init, double RTK_Yaw,
    double walk_distance, double walk_theta, double Walking_Complete_Flag,
    double Avg_Height, double X_Gain, double Theta_Gain,
    double Avg_Height_Plan_, double is_boundary_,
    const std::array<double,2>& Path_,
    double direction_, double walk_num_, double loop_)
{
    PathPlanningOutput output;
    bool cond1 = (Walking_Complete_Flag == 2);
    bool cond2 = (Main_Switch == 1);
    bool cond4 = (is_boundary_ == 0);
    output.Condition_Array = {cond1?1.0:0.0, cond2?1.0:0.0, cond4?1.0:0.0};
    output.is_boundary = is_boundary_;

    if (cond1 && cond2 && cond4) {
        double direction = (direction_ == 0) ? 1 : -direction_;
        double dleta_theta = 0, plan_distance = 0;

        if (direction == 1) {
            double Heading_Plan = Heading_Init;
            dleta_theta = RTK_Yaw - Heading_Plan;
            plan_distance = walk_distance;
            output.walk_num = walk_num_ + 1;
            output.Avg_Height_Plan = Avg_Height + 0.20;
        } else if (direction == -1) {
            double Heading_Plan = Heading_Init - (-walk_theta);
            dleta_theta = RTK_Yaw - Heading_Plan;
            plan_distance = -walk_distance;
            output.walk_num = walk_num_;
            output.Avg_Height_Plan = Avg_Height + 0.60;
        } else {
            output.walk_num = walk_num_;
            output.Avg_Height_Plan = Avg_Height + 0.20;
        }

        output.Path = {dleta_theta * Theta_Gain, plan_distance * X_Gain};
        output.Planning_Complete_Flag = 1;
        output.plan_status = 1;
        output.loop = loop_;
        output.direction = direction;
    } else {
        output.Planning_Complete_Flag = 0;
        output.walk_num = walk_num_;
        output.loop = loop_;
        output.direction = direction_;
        output.Path = Path_;
        output.Avg_Height_Plan = Avg_Height_Plan_;
        output.plan_status = 3;
    }
    return output;
}

} // namespace simulink
#endif // SIMULINK_PATH_H
