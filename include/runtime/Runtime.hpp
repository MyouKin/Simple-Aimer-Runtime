#ifndef AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP
#define AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP

#include "../pipeline/DataProvider.hpp"
#include "../pipeline/System.hpp"
#include "../pipeline/Selector.hpp"
#include "../pipeline/Solver.hpp"
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
            std::shared_ptr<Solver> solver)
        : provider_(provider), system_(system), selector_(selector), solver_(solver), running_(false) {}

    ~Runtime() {
        stop();
    }

    void start() {
        if (running_) return;
        running_ = true;
        
        // Start UI in main thread if needed, or start pipeline in a background thread.
        // For simplicity and to allow ImGui to run in the main thread (which is often required by OS like macOS),
        // we run the pipeline in a background thread and let the caller run ImGui in the main thread.
        pipeline_thread_ = std::thread(&Runtime::pipelineLoop, this);
    }

    void stop() {
        running_ = false;
        if (pipeline_thread_.joinable()) {
            pipeline_thread_.join();
        }
    }

    void runUI() {
        // Run ImGui loop in the current thread (usually main thread)
        debugger_.run();
    }

private:
    void pipelineLoop() {
        while (running_) {
            auto start_time = std::chrono::high_resolution_clock::now();

            InputType input;
            if (provider_->fetch(input)) {
                system_->update(input);
            }

            const auto& state = system_->getState();
            TargetState target = selector_->select(state);
            GimbalCommand cmd = solver_->solve(target);

            // In a real application, you would send `cmd` to the hardware here.

            // Sleep to maintain loop rate (e.g., 100 Hz = 10 ms)
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            if (duration.count() < 10) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10 - duration.count()));
            }
        }
    }

    std::shared_ptr<DataProvider<InputType>> provider_;
    std::shared_ptr<System<InputType, SystemStateType>> system_;
    std::shared_ptr<Selector<SystemStateType>> selector_;
    std::shared_ptr<Solver> solver_;

    std::atomic<bool> running_;
    std::thread pipeline_thread_;
    ImGuiDebugger debugger_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_RUNTIME_RUNTIME_HPP
