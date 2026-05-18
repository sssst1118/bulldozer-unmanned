/**
 * @file log_helper.h
 * @brief 统一日志系统 — 异步写入、按节点分文件、自动滚动
 * @author dozer-dev
 * @date 2026-03-15
 *
 * 使用方式:
 *   1. 在 main() 中 ros::init 之后调用 initLogSystem()
 *   2. 在 main() 退出前调用 destroyLogSystem()
 *   3. 代码中使用 LOG_INFO / LOG_WARN / LOG_ERROR / LOG_DEBUG 宏
 *
 * 日志输出三路并行: 终端 stdout、ROS 日志、节点专属文件
 * 文件位置: <catkin_ws>/logs/<node_name>.log
 *
 * @note 头文件可安全地被多个编译单元 include，不会产生重复定义。
 *       全局变量和非模板函数的实现均在 log_helper.cpp 中。
 */
#pragma once

// 前向声明 Qt 类型 — 不强制依赖 Qt 头文件
#ifndef QT_CORE_QSTRING_H
class QString;
#endif
#ifndef QT_CORE_QBYTEARRAY_H
class QByteArray;
#endif

#include <ros/console.h>
#include <ros/ros.h>

#include <string>
#include <string_view>
#include <sstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <type_traits>
#include <cstdio>
#include <cstdarg>
#include <cstring>
#include <atomic>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>
#include <algorithm>

//==============================================================================
// 日志级别
//==============================================================================
enum LogLevel {
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
};

//==============================================================================
// 配置常量
//==============================================================================
constexpr size_t LOG_MAX_SIZE    = 100UL * 1024 * 1024;  ///< 单文件上限 100MB
constexpr int    LOG_MAX_BACKUPS = 5;                     ///< 滚动备份数量

//==============================================================================
// 日志条目
//==============================================================================
struct LogItem {
    std::string content;
    LogLevel    level;
    std::string timestamp;
    std::string file;
    int         line;
    std::string func;
};

//==============================================================================
// 全局状态 — extern 声明，定义在 log_helper.cpp
//==============================================================================
extern std::atomic<bool> ENABLE_LOG_DEBUG;
extern std::atomic<bool> ENABLE_LOG_INFO;
extern std::atomic<bool> ENABLE_LOG_WARN;
extern std::atomic<bool> ENABLE_LOG_ERROR;

extern std::queue<LogItem> g_log_queue;
extern std::mutex          g_log_mutex;
extern std::thread         g_log_thread;
extern std::atomic<bool>   g_log_running;

//==============================================================================
// 生命周期管理 — 实现在 log_helper.cpp
//==============================================================================

/** @brief 初始化日志系统 (启动写入线程、创建日志目录), 在 ros::init 之后调用 */
void initLogSystem();

/** @brief 销毁日志系统 (排空队列、等待写入线程结束), 在 main 退出前调用 */
void destroyLogSystem();

/** @brief 排空日志队列 (阻塞等待至多1秒) */
void flushLogQueue();

//==============================================================================
// 轻量内联工具
//==============================================================================

/** @brief 从完整路径中提取文件名 */
inline std::string getFileName(const std::string& full_path) {
    const char* p = std::strrchr(full_path.c_str(), '/');
    return p ? std::string(p + 1) : full_path;
}

/** @brief 日志级别转字符串 */
inline const char* getLogLevelStr(LogLevel level) {
    switch (level) {
        case LOG_LEVEL_DEBUG: return "DEBUG";
        case LOG_LEVEL_INFO:  return "INFO";
        case LOG_LEVEL_WARN:  return "WARN";
        case LOG_LEVEL_ERROR: return "ERROR";
        default:              return "UNKNOWN";
    }
}

/** @brief 获取当前 ROS 节点名 (斜杠替换为下划线, 用于日志文件名) */
inline std::string getRosNodeName() {
    if (ros::isInitialized()) {
        std::string name = ros::this_node::getName();
        std::replace(name.begin(), name.end(), '/', '_');
        return name;
    }
    return "unknown_node";
}

/** @brief 生成毫秒级时间戳字符串 "YYYY-MM-DD HH:MM:SS.mmm" */
inline std::string getTimestamp() {
    auto now    = std::chrono::system_clock::now();
    auto ms     = std::chrono::duration_cast<std::chrono::milliseconds>(
                      now.time_since_epoch()) % 1000;
    auto time_t_now = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d %H:%M:%S")
       << "." << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

//==============================================================================
// 类型转换
//==============================================================================

/// Qt 类型转换 — 实现在 log_helper.cpp (避免头文件依赖 Qt)
std::string qstringToStdString(const QString& qstr);
std::string qbyteArrayToStdString(const QByteArray& qba);

/** @brief 将任意类型转为 std::string, 支持基本类型、字符串、Qt 类型 */
template <typename T>
inline std::string toString(const T& value) {
    if constexpr (std::is_same_v<T, const char*> || std::is_same_v<T, char*>) {
        return std::string(value);
    } else if constexpr (std::is_same_v<T, std::string>) {
        return value;
    } else if constexpr (std::is_same_v<T, std::string_view>) {
        return std::string(value);
    } else if constexpr (std::is_arithmetic_v<T>) {
        std::stringstream ss;
        ss << value;
        return ss.str();
    } else if constexpr (std::is_same_v<T, QString>) {
        return qstringToStdString(value);
    } else if constexpr (std::is_same_v<T, QByteArray>) {
        return qbyteArrayToStdString(value);
    } else {
        return "[Unknown Type]";
    }
}

//==============================================================================
// 格式化与入队
//==============================================================================

/**
 * @brief printf 风格格式化 (线程安全, 使用 thread_local 缓冲区)
 * @return 格式化后的 C 字符串, 在同一线程下次调用前有效
 */
inline const char* log_format(const char* fmt, ...) {
    va_list args;

    va_start(args, fmt);
    int len = vsnprintf(nullptr, 0, fmt, args) + 1;
    va_end(args);

    static thread_local std::vector<char> buf;
    buf.resize(static_cast<size_t>(len));

    va_start(args, fmt);
    vsnprintf(buf.data(), buf.size(), fmt, args);
    va_end(args);

    return buf.data();
}

/**
 * @brief 日志入队 — 组装日志条目并推入异步写入队列
 * @note 模板函数, 编译器按需实例化, 不会产生重复符号
 */
template <typename T>
void logEnqueue(LogLevel level, const char* file, int line, const char* func, const T& msg) {
    // 级别开关过滤
    if ((level == LOG_LEVEL_DEBUG && !ENABLE_LOG_DEBUG) ||
        (level == LOG_LEVEL_INFO  && !ENABLE_LOG_INFO)  ||
        (level == LOG_LEVEL_WARN  && !ENABLE_LOG_WARN)  ||
        (level == LOG_LEVEL_ERROR && !ENABLE_LOG_ERROR)) {
        return;
    }

    std::string logContent = toString(msg);
    std::string fileName   = getFileName(file);
    std::string timestamp  = getTimestamp();

    std::string finalLog = timestamp
        + " [" + getLogLevelStr(level) + "] "
        + "[" + fileName + ":" + std::to_string(line) + "] "
        + "[" + func + "] "
        + logContent;

    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_queue.push({finalLog, level, timestamp, fileName, line, func});
}

//==============================================================================
// 日志宏 — 用户接口
//==============================================================================
#define LOG_DEBUG(...) logEnqueue(LOG_LEVEL_DEBUG, __FILE__, __LINE__, __func__, log_format(__VA_ARGS__))
#define LOG_INFO(...)  logEnqueue(LOG_LEVEL_INFO,  __FILE__, __LINE__, __func__, log_format(__VA_ARGS__))
#define LOG_WARN(...)  logEnqueue(LOG_LEVEL_WARN,  __FILE__, __LINE__, __func__, log_format(__VA_ARGS__))
#define LOG_ERROR(...) logEnqueue(LOG_LEVEL_ERROR, __FILE__, __LINE__, __func__, log_format(__VA_ARGS__))
