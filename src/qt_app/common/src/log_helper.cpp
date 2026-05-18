/**
 * @file log_helper.cpp
 * @brief 日志系统实现 — 全局变量定义、异步写入线程、文件管理
 * @author dozer-dev
 * @date 2026-03-15
 */

#include "qt_app/log_helper.h"

#include <QString>
#include <QByteArray>

#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <sys/stat.h>

//==============================================================================
// 全局变量定义 (唯一定义点, 头文件中为 extern 声明)
//==============================================================================
std::atomic<bool> ENABLE_LOG_DEBUG(true);
std::atomic<bool> ENABLE_LOG_INFO(true);
std::atomic<bool> ENABLE_LOG_WARN(true);
std::atomic<bool> ENABLE_LOG_ERROR(true);

std::queue<LogItem> g_log_queue;
std::mutex          g_log_mutex;
std::thread         g_log_thread;
std::atomic<bool>   g_log_running(false);

//==============================================================================
// 内部辅助 (匿名命名空间, 仅本文件可见)
//==============================================================================
namespace {

/** @brief 日志脱敏 — 掩盖手机号等敏感信息 */
std::string desensitizeLog(const std::string& content) {
    std::string res = content;
    size_t pos = res.find("phone:");
    if (pos != std::string::npos && res.length() >= pos + 17) {
        res.replace(pos + 6, 11, "***********");
    }
    return res;
}

/**
 * @brief 动态推断日志基础目录
 * @return 日志目录路径, 优先级: CMAKE_PREFIX_PATH > ROS_PACKAGE_PATH > cwd > /tmp
 */
std::string getLogBasePath() {
    // 方法1: 从 CMAKE_PREFIX_PATH 中提取 catkin_ws 路径
    const char* cmake_prefix = std::getenv("CMAKE_PREFIX_PATH");
    if (cmake_prefix) {
        std::string prefix_str(cmake_prefix);
        size_t devel_pos = prefix_str.find("/devel");
        if (devel_pos != std::string::npos) {
            std::string ws_path = prefix_str.substr(0, devel_pos);
            size_t colon_pos = ws_path.rfind(':');
            if (colon_pos != std::string::npos) {
                ws_path = ws_path.substr(colon_pos + 1);
            }
            return ws_path + "/logs";
        }
    }

    // 方法2: 从 ROS_PACKAGE_PATH 中提取
    const char* ros_pkg_path = std::getenv("ROS_PACKAGE_PATH");
    if (ros_pkg_path) {
        std::string pkg_str(ros_pkg_path);
        size_t src_pos = pkg_str.find("/src");
        if (src_pos != std::string::npos) {
            std::string ws_path = pkg_str.substr(0, src_pos);
            size_t colon_pos = ws_path.rfind(':');
            if (colon_pos != std::string::npos) {
                ws_path = ws_path.substr(colon_pos + 1);
            }
            return ws_path + "/logs";
        }
    }

    // 方法3: 从当前工作目录推断
    char cwd[1024];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::string cwd_str(cwd);
        for (const char* pattern : {"/catkin_ws", "/ros_ws", "/bulldozer-unmanned"}) {
            size_t ws_pos = cwd_str.rfind(pattern);
            if (ws_pos != std::string::npos) {
                size_t end_pos = cwd_str.find('/', ws_pos + 1);
                if (end_pos == std::string::npos) end_pos = cwd_str.length();
                return cwd_str.substr(0, end_pos) + "/logs";
            }
        }
    }

    return "/tmp/ros_logs";
}

/** @brief 创建日志目录 (只在 init 时调用一次) */
void ensureLogDirExists() {
    std::string log_dir = getLogBasePath();
    // 使用 POSIX mkdir 代替 system() 调用, 避免 fork 开销
    mkdir(log_dir.c_str(), 0777);
}

/** @brief 获取节点日志文件完整路径 */
std::string getLogFilePath() {
    return getLogBasePath() + "/" + getRosNodeName() + ".log";
}

/** @brief 日志文件滚动 — 超过 LOG_MAX_SIZE 时重命名备份 */
void checkLogRotation(const std::string& log_path) {
    struct stat st;
    if (stat(log_path.c_str(), &st) != 0) return;
    if (static_cast<size_t>(st.st_size) < LOG_MAX_SIZE) return;

    // 滚动: node.log.4 → node.log.5, ... , node.log → node.log.1
    for (int i = LOG_MAX_BACKUPS - 1; i > 0; --i) {
        std::string old_name = log_path + "." + std::to_string(i);
        std::string new_name = log_path + "." + std::to_string(i + 1);
        std::rename(old_name.c_str(), new_name.c_str());
    }
    std::rename(log_path.c_str(), (log_path + ".1").c_str());
}

/** @brief 异步写入线程主循环 */
void logWriterThread() {
    // 持有文件句柄, 避免每条日志都 open/close
    std::string current_log_path;
    std::ofstream log_file;
    int write_count = 0;

    while (g_log_running) {
        std::unique_lock<std::mutex> lock(g_log_mutex);
        if (g_log_queue.empty()) {
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        LogItem item = std::move(g_log_queue.front());
        g_log_queue.pop();
        lock.unlock();

        // 1. 终端输出
        std::fputs(item.content.c_str(), stdout);
        std::fputc('\n', stdout);

        // 2. ROS 日志输出
        if (ros::isInitialized()) {
            std::string ros_msg = "[" + item.file + ":" + std::to_string(item.line)
                                + "] [" + item.func + "] "
                                + desensitizeLog(item.content);
            switch (item.level) {
                case LOG_LEVEL_DEBUG: ROS_DEBUG("%s", ros_msg.c_str()); break;
                case LOG_LEVEL_INFO:  ROS_INFO("%s",  ros_msg.c_str()); break;
                case LOG_LEVEL_WARN:  ROS_WARN("%s",  ros_msg.c_str()); break;
                case LOG_LEVEL_ERROR: ROS_ERROR("%s", ros_msg.c_str()); break;
            }
        }

        // 3. 文件写入
        std::string log_path = getLogFilePath();

        // 如果文件路径变了(不太可能)或文件未打开, 重新打开
        if (!log_file.is_open() || log_path != current_log_path) {
            if (log_file.is_open()) log_file.close();
            current_log_path = log_path;
            log_file.open(current_log_path, std::ios::app | std::ios::out);
            if (!log_file.is_open()) {
                std::fprintf(stderr, "[LOG_ERROR] 无法打开日志文件: %s\n", current_log_path.c_str());
                continue;
            }
        }

        log_file << desensitizeLog(item.content) << '\n';

        // 每100条刷盘一次 + 检查滚动, 平衡性能与可靠性
        if (++write_count >= 100) {
            write_count = 0;
            log_file.flush();
            checkLogRotation(current_log_path);
        }
    }

    // 退出前刷盘
    if (log_file.is_open()) {
        log_file.flush();
        log_file.close();
    }
}

}  // anonymous namespace

//==============================================================================
// 公开接口实现
//==============================================================================

void initLogSystem() {
    // 设置编码环境
    setenv("LC_ALL", "zh_CN.UTF-8", 1);
    setenv("LANG", "zh_CN.UTF-8", 1);
    setenv("ROSCONSOLE_FORMAT", "[${severity}] [${time}] [${file}:${line}]: ${message}", 1);

    // 只在启动时创建一次目录
    ensureLogDirExists();

    g_log_running = true;
    g_log_thread = std::thread(logWriterThread);
}

void destroyLogSystem() {
    flushLogQueue();
    g_log_running = false;
    if (g_log_thread.joinable()) {
        g_log_thread.join();
    }
}

void flushLogQueue() {
    int wait_ms = 0;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(g_log_mutex);
            if (g_log_queue.empty()) break;
        }
        if (wait_ms >= 1000) break;  // 最多等1秒
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        wait_ms += 10;
    }
}

//==============================================================================
// Qt 类型转换
//==============================================================================

std::string qstringToStdString(const QString& qstr) {
    QByteArray utf8_ba = qstr.toUtf8();
    return std::string(utf8_ba.constData(), utf8_ba.size());
}

std::string qbyteArrayToStdString(const QByteArray& qba) {
    return std::string(qba.constData(), qba.size());
}
