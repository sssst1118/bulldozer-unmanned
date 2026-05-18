/**
 * @file path_generator.h
 * @brief 路径生成器 — 符合推土机运动学约束的路径点队列
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 两种工艺:
 *   ZIGZAG:         推到末端 → 从末端斜着提刀到下一列起始端 → 推
 *                   (顶端斜移换列, 无需倒车, 一步换列)
 *   UNIDIRECTIONAL: 推到末端 → 提刀倒回起始端 → 从起始端斜着到下一列起始端 → 推
 *                   (底端斜移换列, 倒车后短距离换列)
 *
 * 支持每列多遍 (passes_per_col): 每列重复推多遍再换列
 * 无地图模式直接指定列数 (num_columns), 有地图模式用 calcNumPasses()
 *
 * 路径点使用相对于作业起点的 ENU 坐标。
 * 推土方向由 heading 参数控制, 内部转换为局部坐标系:
 *   局部X轴 = 推土方向 (heading方向, forward)
 *   局部Y轴 = 推土方向的左侧 (换列方向, lateral)
 *
 * @par 铲刀目标高度 (target_height):
 *   本文件在生成路径点时, 顺便把每个点的 target_height 填好:
 *     RAISE 点 → target_height = RAISE_HEIGHT        (提刀)
 *     LEVEL 点 → target_height = computeLevelHeight() (按 level_mode 算)
 *   执行器 (decision_system) 取该值下发给 moldboard_controller。
 *   本层不关心下游怎么实现跟踪, 只负责把"目标高度"算出来填进去。
 */
#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "path_types.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace path_gen {

/**
 * @brief 将局部坐标 (沿推土方向/垂直方向) 转换为 ENU 坐标
 */
inline void localToENU(double local_forward, double local_lateral,
                       double heading_deg, double origin_x, double origin_y,
                       double& enu_x, double& enu_y) {
    double rad = heading_deg * M_PI / 180.0;
    enu_x = origin_x + local_forward * std::sin(rad) + local_lateral * (-std::cos(rad));
    enu_y = origin_y + local_forward * std::cos(rad) + local_lateral * std::sin(rad);
}

/**
 * @brief 根据 LevelMode 计算一个 LEVEL 路径点的目标高度
 * @param params    路径生成参数 (提供 level_mode 及对应参数)
 * @param local_fwd 该点在"推土方向"上的局部坐标 (米), SLOPE_3D 用
 * @return 目标高度 (米, ENU-Up 绝对高程)
 *
 * @par 当前实现:
 *   - FLAT_3D  → 直接返回 target_level_height
 *   - SLOPE_3D → slope_start_height + local_fwd × slope_gradient / 100
 */
inline double computeLevelHeight(const PathGenParams& params,
                                 double local_fwd) {
    switch (params.level_mode) {
        case LevelMode::FLAT_3D:
            return params.target_level_height;
        case LevelMode::SLOPE_3D:
            return params.slope_start_height +
                   local_fwd * params.slope_gradient / 100.0;
        default:
            return params.target_level_height;
    }
}

/**
 * @brief 生成 Z字推路径 — 顶端斜移换列, 支持每列多遍
 */
inline std::vector<WayPoint> generateZigzagPath(const PathGenParams& params,
                                                 bool use_explicit_cols = false) {
    std::vector<WayPoint> path;

    double ox = params.start_x;
    double oy = params.start_y;
    double h  = params.heading;
    int num_cols = use_explicit_cols ? params.num_columns : params.calcNumPasses();
    int ppc = std::max(1, params.passes_per_col);

    for (int i = 0; i < num_cols; ++i) {
        double lateral = i * params.blade_width;

        double start_ex, start_ey, end_ex, end_ey;
        localToENU(0, lateral, h, ox, oy, start_ex, start_ey);
        localToENU(params.push_length, lateral, h, ox, oy, end_ex, end_ey);

        for (int p = 0; p < ppc; ++p) {
            bool last_pass = (p == ppc - 1);

            // [Fix-BUG-H] 推土终点: 目标高度按该点的 local_fwd 位置计算
            double h_level = computeLevelHeight(params, params.push_length);
            path.emplace_back(end_ex, end_ey, BladeCmd::LEVEL, DriveDir::FORWARD,
                              h_level);

            if (!last_pass) {
                // 非最后一遍: 倒车回 bot, 提刀
                path.emplace_back(start_ex, start_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                                  RAISE_HEIGHT);
            }
        }

        // 最后一遍推完后: 从 top 斜移到下一列 bot, 提刀
        if (i < num_cols - 1) {
            double next_lateral = (i + 1) * params.blade_width;
            double shift_ex, shift_ey;
            localToENU(0, next_lateral, h, ox, oy, shift_ex, shift_ey);
            path.emplace_back(shift_ex, shift_ey, BladeCmd::RAISE, DriveDir::FORWARD,
                              RAISE_HEIGHT);
        }
    }

    return path;
}

/**
 * @brief 生成单向推路径 — 倒车回底端 + 底端斜移换列, 支持每列多遍
 */
inline std::vector<WayPoint> generateUnidirectionalPath(const PathGenParams& params,
                                                         bool use_explicit_cols = false) {
    std::vector<WayPoint> path;

    double ox = params.start_x;
    double oy = params.start_y;
    double h  = params.heading;
    int num_cols = use_explicit_cols ? params.num_columns : params.calcNumPasses();
    int ppc = std::max(1, params.passes_per_col);

    // 斜移前进量: blade_width / tan(angle)
    double angle_rad = std::max(params.shift_angle, 1.0) * M_PI / 180.0;
    double fwd_offset = params.blade_width / std::tan(angle_rad);

    for (int i = 0; i < num_cols; ++i) {
        double lateral = i * params.blade_width;

        double start_ex, start_ey, end_ex, end_ey;
        localToENU(0, lateral, h, ox, oy, start_ex, start_ey);
        localToENU(params.push_length, lateral, h, ox, oy, end_ex, end_ey);

        for (int p = 0; p < ppc; ++p) {
            // [Fix-BUG-H] 推土终点: 目标高度按 push_length 位置计算
            double h_level = computeLevelHeight(params, params.push_length);
            path.emplace_back(end_ex, end_ey, BladeCmd::LEVEL, DriveDir::FORWARD,
                              h_level);
            // 倒车: top → bot (提刀, 同列直线)
            path.emplace_back(start_ex, start_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                              RAISE_HEIGHT);
        }

        // 所有遍完成, 斜移到下一列
        if (i < num_cols - 1) {
            double next_lateral = (i + 1) * params.blade_width;

            // 斜着前进到 (fwd_offset, next_col), 提刀
            double diag_ex, diag_ey;
            localToENU(fwd_offset, next_lateral, h, ox, oy, diag_ex, diag_ey);
            path.emplace_back(diag_ex, diag_ey, BladeCmd::RAISE, DriveDir::FORWARD,
                              RAISE_HEIGHT);

            // 短倒车回下一列底端 (0, next_col), 提刀
            double next_start_ex, next_start_ey;
            localToENU(0, next_lateral, h, ox, oy, next_start_ex, next_start_ey);
            path.emplace_back(next_start_ex, next_start_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                              RAISE_HEIGHT);
        }
    }

    return path;
}

/**
 * @brief 生成底端倒车斜移路径 — 倒车回底端后直接斜着倒车到下一列, 一步换列
 */
inline std::vector<WayPoint> generateUnidiBackShiftPath(const PathGenParams& params,
                                                         bool use_explicit_cols = false) {
    std::vector<WayPoint> path;

    double ox = params.start_x;
    double oy = params.start_y;
    double h  = params.heading;
    int num_cols = use_explicit_cols ? params.num_columns : params.calcNumPasses();
    int ppc = std::max(1, params.passes_per_col);

    // 斜移后退量: blade_width / tan(angle)
    double angle_rad = std::max(params.shift_angle, 1.0) * M_PI / 180.0;
    double fwd_offset = params.blade_width / std::tan(angle_rad);

    for (int i = 0; i < num_cols; ++i) {
        double lateral = i * params.blade_width;

        // 第一列从 (0, 0) 开始推, 后续列从 (-fwd_offset, lateral) 开始推
        double push_start = (i == 0) ? 0.0 : -fwd_offset;

        double start_ex, start_ey, end_ex, end_ey;
        localToENU(push_start, lateral, h, ox, oy, start_ex, start_ey);
        localToENU(params.push_length, lateral, h, ox, oy, end_ex, end_ey);

        for (int p = 0; p < ppc; ++p) {
            bool last_pass = (p == ppc - 1);

            // [Fix-BUG-H] 推土终点: 目标高度按 push_length 位置计算
            // SLOPE_3D 下目标高度 = slope_start + push_length * gradient/100
            // 注意: 执行阶段 execDriving 会按实际行驶距离实时更新, 这里只设终点值
            double h_level = computeLevelHeight(params, params.push_length);
            path.emplace_back(end_ex, end_ey, BladeCmd::LEVEL, DriveDir::FORWARD,
                              h_level);

            if (!last_pass) {
                // 非最后一遍: 倒车回本列起点, 提刀
                path.emplace_back(start_ex, start_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                                  RAISE_HEIGHT);
            }
        }

        // 最后一遍推完后换列
        if (i < num_cols - 1) {
            // 先直线倒车回本列底端 (0, col), 提刀
            double bot_ex, bot_ey;
            localToENU(0, lateral, h, ox, oy, bot_ex, bot_ey);
            path.emplace_back(bot_ex, bot_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                              RAISE_HEIGHT);

            // 斜着倒车到下一列后方 (-fwd_offset, next_col), 提刀
            double next_lateral = (i + 1) * params.blade_width;
            double shift_ex, shift_ey;
            localToENU(-fwd_offset, next_lateral, h, ox, oy, shift_ex, shift_ey);
            path.emplace_back(shift_ex, shift_ey, BladeCmd::RAISE, DriveDir::BACKWARD,
                              RAISE_HEIGHT);
        }
    }

    return path;
}

/**
 * @brief 根据工艺类型生成路径
 * @param use_explicit_cols true=无地图模式(用num_columns), false=有地图模式(用calcNumPasses)
 */
inline std::vector<WayPoint> generatePath(PathMode mode, const PathGenParams& params,
                                           bool use_explicit_cols = false) {
    switch (mode) {
        case PathMode::ZIGZAG:            return generateZigzagPath(params, use_explicit_cols);
        case PathMode::UNIDIRECTIONAL:    return generateUnidirectionalPath(params, use_explicit_cols);
        case PathMode::UNIDI_BACK_SHIFT:  return generateUnidiBackShiftPath(params, use_explicit_cols);
        default:                          return {};
    }
}

}  // namespace path_gen

#endif // PATH_GENERATOR_H
