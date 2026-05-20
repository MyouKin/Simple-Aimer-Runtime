#ifndef AIM_FRAMEWORK_DEBUG_AIMER_CURVE_HPP
#define AIM_FRAMEWORK_DEBUG_AIMER_CURVE_HPP

/// @file AimerCurve.hpp
/// @brief 独立曲线窗口 —— 多实例，支持多坐标轴，每个轴可选择显示哪些曲线
///
/// 用法：
///   AimerCurve curves("Trajectory");
///   curves.push("yaw",   1.23f);   // 线程安全
///   curves.push("pitch", 0.45f);
///   curves.openWindow();           // 打开独立 OS 窗口，用户自由添加坐标轴和分配曲线

#include <string>
#include <vector>
#include <deque>
#include <unordered_map>
#include <mutex>
#include <thread>
#include <atomic>

struct GLFWwindow;

namespace aim {

/// @brief 坐标轴定义（在窗口中由用户自由创建和管理）
struct CurveAxis {
    char   name[64] = "";
    std::vector<std::string> curves; ///< 本轴绑定的曲线名列表
    bool   visible = true;
};

class AimerCurve {
public:
    explicit AimerCurve(const std::string& name);
    ~AimerCurve();

    /// @brief 推送一个数据点（线程安全，可从 pipeline 线程调用）
    void push(const std::string& curve_name, float value);

    /// @brief 窗口控制
    void openWindow();
    void closeWindow();
    bool isWindowOpen() const;

    const std::string& name() const { return name_; }

private:
    void windowThread();

    std::string name_;
    size_t max_points_ = 1000;

    std::unordered_map<std::string, std::deque<float>> curves_;
    std::vector<CurveAxis> axes_;
    std::unordered_map<std::string, bool> curve_visible_;
    std::mutex mutex_;

    std::thread thread_;
    std::atomic<bool> running_{false};
    GLFWwindow* window_ = nullptr;
};

} // namespace aim
#endif
