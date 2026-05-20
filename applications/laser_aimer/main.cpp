#include "../../include/runtime/Runtime.hpp"
#include "../../include/debug/AimerLogger.hpp"
#include "../../include/debug/AimerImage.hpp"
#include "../../include/debug/AimerCurve.hpp"
#include "LaserAimerProvider.hpp"
#include "../../include/pipeline/Selector.hpp"
#include "../../include/pipeline/System.hpp"
#include "LaserAimerSolver.hpp"

#include <iostream>

using namespace aim;

// We need a simple Selector since LaserAimerProvider directly outputs std::optional<aim::FinalTargetState>
class DirectTargetSelector : public Selector<std::optional<aim::FinalTargetState>> {
public:
    aim::FinalTargetState select(const std::optional<aim::FinalTargetState>& system_state) override {
        if (system_state.has_value()) {
            return system_state.value();
        }
        aim::FinalTargetState empty;
        empty.valid = false;
        return empty;
    }
};

int main() {
    std::cout << "Starting Laser Aimer Application...\n";

    // 1. Data Provider
    auto provider = std::make_shared<LaserAimerProvider>();
    
    // 2. System: Just hold the TargetState since it's a single target mode for now
    class DirectSystem : public System<std::optional<aim::FinalTargetState>, std::optional<aim::FinalTargetState>> {
        std::optional<aim::FinalTargetState> state_;
    public:
        void update(const std::optional<aim::FinalTargetState>& input) override { state_ = input; }
        const std::optional<aim::FinalTargetState>& getState() const override { return state_; }
    };
    auto system = std::make_shared<DirectSystem>();
    
    // 3. Selector: Extracts the state directly
    auto selector = std::make_shared<DirectTargetSelector>();
    
    // 4. Solver: Use Boresight Offset Parallax Logic + P Controller
    auto solver = std::make_shared<LaserAimerSolver>();

    Runtime<std::optional<aim::FinalTargetState>, std::optional<aim::FinalTargetState>> runtime(provider, system, selector, solver);

    // ---- 启动管线 ----
    runtime.start();
    std::cout << "Pipeline running. Press Ctrl+C to stop.\n";

    // ---- Debug 窗口（独立 OS 窗口，按需打开） ----
    auto& logger = aim::AimerLogger::instance();
    logger.showWindow();  // 打开日志窗口

    // 示例：图像窗口（Provider 可推送图像到此处）
    // aim::AimerImage laser_view("Laser View");
    // laser_view.openWindow();

    // 示例：曲线窗口
    // aim::AimerCurve controls("Control Signals");
    // controls.openWindow();

    // ---- 主线程阻塞等待退出 ----
    aim::waitForShutdown();

    runtime.stop();
    std::cout << "Exiting...\n";
    return 0;
}