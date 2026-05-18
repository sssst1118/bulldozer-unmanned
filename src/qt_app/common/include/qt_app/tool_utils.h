/**
 * @file tool_utils.h
 * @brief Tool Utils
 * @author dozer-dev
 * @date 2026-03-15
 */
#ifndef QT_APP_TOOL_UTILS_H
#define QT_APP_TOOL_UTILS_H

#include <cstdint>

// 获取当前系统的毫秒级时间戳（从1970年开始，工控机多模块同步用）
uint64_t get_current_ms();

// 数值限幅函数（比如履带速度不能超过最大值，铲刀高度不能超出限位）
template <typename T>
T clamp(T value, T min_val, T max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#endif // QT_APP_TOOL_UTILS_H
