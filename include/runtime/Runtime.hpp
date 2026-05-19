#ifndef AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP
#define AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP

#include "../pipeline/DataProvider.hpp"
#include "../pipeline/System.hpp"
#include "../pipeline/Selector.hpp"
#include "../pipeline/Solver.hpp"
#include "../pipeline/Actuator.hpp"
#include "../core/FrameTimer.hpp"
#include "../debug/ImGuiDebugger.hpp"

#include <memory>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

namespace aim {

template <typename InputType, typename SystemStateType>
class Runtime {
public:
    Runtime(std::shared_ptr<DataProvider<InputType>> provider,
            std::shared_ptr<System<InputType, SystemStateType>> system,
            std::shared_ptr<Selector<SystemStateType>> selector,
            std::shared_ptr<Solver<SystemStateType>> solver,
            std::shared_ptr<Actuator> actuator = nullptr,
            double loop_rate_hz = 100.0)
        : provider_(std::move(provider))
        , system_(std::move(system))
        , selector_(std::move(selector))
        , solver_(std::move(solver))
        , actuator_(std::move(actuator))
        , loop_period_ms_(loop_rate_hz > 0.0 ? (1000.0 / loop_rate_hz) : 10.0)
        , running_(false) {}

    ~Runtime() {
        stop();
    }

    void start() {
        if (running_) return;
        running_ = true;

        pipeline_thread_ = std::thread(&Runtime::pipelineLoop, this);
    }

    void stop() {
        running_ = false;
        if (pipeline_thread_.joinable()) {
            pipeline_thread_.join();
        }
    }

    void runUI() {
        debugger_.run();
    }

    /// @brief 获取平滑后的帧间隔（秒），可用于外部监控
    double frameDt() const { return frame_timer_.dt(); }

private:
    void pipelineLoop() {
        frame_timer_.reset();

        while (running_) {
            // ---- 0. 取最新反馈（非阻塞，Actuator 内部线程持续更新） ----
            std::optional<SelfState> fb;
            if (actuator_) fb = actuator_->feedback();
            if (fb.has_value()) provider_->acceptFeedback(*fb);

            // ---- 1. 数据采集 ----
            InputType input;
            bool has_input = provider_->fetch(input);

            if (has_input) {
                // ---- 2. 系统状态更新 ----
                system_->update(input);
            }
            if (fb.has_value()) system_->updateSelfState(*fb);

            // ---- 3. 目标选择 ----
            const auto& state = system_->getState();
            FinalTargetState target = selector_->select(state);
            Command cmd = solver_->solve(target, state);

            // ---- 4. 发送指令到硬件 ----
            if (actuator_) actuator_->send(cmd);

            // ---- 7. 帧率控制 ----
            auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::high_resolution_clock::now() - frame_timer_.lastTickPoint()).count();
            double remaining_ms = loop_period_ms_ - elapsed;
            if (remaining_ms > 0.5) {
                std::this_thread::sleep_for(
                    std::chrono::microseconds(static_cast<long long>(remaining_ms * 1000.0)));
            }
        }
    }

    std::shared_ptr<DataProvider<InputType>> provider_;
    std::shared_ptr<System<InputType, SystemStateType>> system_;
    std::shared_ptr<Selector<SystemStateType>> selector_;
    std::shared_ptr<Solver<SystemStateType>> solver_;
    std::shared_ptr<Actuator> actuator_;

    double loop_period_ms_;
    std::atomic<bool> running_;
    std::thread pipeline_thread_;
    ImGuiDebugger debugger_;
    FrameTimer frame_timer_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP
