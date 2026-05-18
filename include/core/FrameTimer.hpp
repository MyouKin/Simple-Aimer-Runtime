#ifndef AIM_FRAMEWORK_CORE_FRAME_TIMER_HPP
#define AIM_FRAMEWORK_CORE_FRAME_TIMER_HPP

#include <chrono>

namespace aim {

/// @brief 轻量帧计时器 — 跟踪帧间 dt，提供平滑后的时间差。
///
/// 用法：
///   FrameTimer timer;
///   while (running) {
///       double dt = timer.tick();  // 返回本帧距上一帧的秒数
///       // ... 管道循环 ...
///   }
class FrameTimer {
public:
    using Clock = std::chrono::high_resolution_clock;

    FrameTimer(double smoothing_alpha = 0.1)
        : smoothing_alpha_(smoothing_alpha), last_tick_(Clock::now()) {}

    /// @brief 标记新一帧开始，返回距上一帧的秒数（平滑后）
    double tick() {
        auto now = Clock::now();
        double raw_dt = std::chrono::duration<double>(now - last_tick_).count();
        last_tick_ = now;

        if (raw_dt <= 0.0) raw_dt = 1e-6;

        if (smoothed_dt_ <= 0.0) {
            smoothed_dt_ = raw_dt;
        } else {
            smoothed_dt_ = smoothing_alpha_ * raw_dt + (1.0 - smoothing_alpha_) * smoothed_dt_;
        }
        return smoothed_dt_;
    }

    /// @brief 获取平滑后的 dt（不推进计时）
    double dt() const { return smoothed_dt_; }

    /// @brief 获取上一次 tick 的时间点（用于帧率控制等）
    Clock::time_point lastTickPoint() const { return last_tick_; }

    /// @brief 重置计时器
    void reset() {
        last_tick_ = Clock::now();
        smoothed_dt_ = 0.0;
    }

private:
    double smoothing_alpha_;
    double smoothed_dt_ = 0.0;
    Clock::time_point last_tick_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_CORE_FRAME_TIMER_HPP