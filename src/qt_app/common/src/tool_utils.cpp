/**
 * @file tool_utils.cpp
 * @brief Tool Utils
 * @author dozer-dev
 * @date 2026-03-15
 */
#include "qt_app/tool_utils.h"
#include <chrono>  

// 实现毫秒级时间戳
uint64_t get_current_ms() {
    // 获取当前系统时间，转换为毫秒
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}
