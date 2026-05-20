#ifndef AIM_FRAMEWORK_DEBUG_AIMER_LOGGER_HPP
#define AIM_FRAMEWORK_DEBUG_AIMER_LOGGER_HPP

/// @file AimerLogger.hpp
/// @brief 独立日志窗口 —— 基于 spdlog 环形缓冲，线程安全，支持文本复制
///
/// 用法：
///   AimerLogger::instance().info("hello {}", 42);
///   AimerLogger::instance().showWindow();
///
/// 日志窗口是独立的操作系统窗口，大小/位置由 imgui.ini 自动记忆。

#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>

struct GLFWwindow;

// 前向声明 spdlog（避免强制包含）
namespace spdlog {
class logger;
namespace level {
enum level_enum : int;
}
}

namespace aim {

class AimerLogger {
public:
    static AimerLogger& instance();

    // ---- 简洁日志接口（应用层直接调用） ----
    void info(const std::string& msg);
    void warn(const std::string& msg);
    void error(const std::string& msg);
    void debug(const std::string& msg);

    /// @brief 获取底层 spdlog::logger，可用于 SPDLOG_INFO 等宏
    std::shared_ptr<spdlog::logger> spdlogLogger();

    // ---- 窗口控制 ----
    void showWindow();
    void hideWindow();
    bool isWindowOpen() const;

private:
    AimerLogger();
    ~AimerLogger();

    struct Entry {
        int level = 2;  // spdlog level enum: 0=trace..6=off
        std::string text;
        std::chrono::system_clock::time_point time;
    };

    void windowThread();

    std::vector<Entry> buffer_;
    size_t buffer_max_ = 2000;
    std::mutex mutex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    GLFWwindow* window_ = nullptr;

    std::shared_ptr<spdlog::logger> spdlog_logger_;
};

} // namespace aim
#endif
