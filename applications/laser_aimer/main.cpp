#include "../../include/runtime/Runtime.hpp"
#include "LaserAimerProvider.hpp"
#include "../../include/pipeline/Selector.hpp"
#include "../../include/pipeline/System.hpp"
#include "LaserAimerSolver.hpp"

#include <iostream>

using namespace aim;

// We need a simple Selector since LaserAimerProvider directly outputs std::optional<TargetState>
class DirectTargetSelector : public Selector<std::optional<TargetState>> {
public:
    TargetState select(const std::optional<TargetState>& system_state) override {
        if (system_state.has_value()) {
            return system_state.value();
        }
        TargetState empty;
        empty.valid = false;
        return empty;
    }
};

int main() {
    std::cout << "Starting Laser Aimer Application...\n";

    // 1. Data Provider
    auto provider = std::make_shared<LaserAimerProvider>();
    
    // 2. System: Just hold the TargetState since it's a single target mode for now
    class DirectSystem : public System<std::optional<TargetState>, std::optional<TargetState>> {
        std::optional<TargetState> state_;
    public:
        void update(const std::optional<TargetState>& input) override { state_ = input; }
        const std::optional<TargetState>& getState() const override { return state_; }
    };
    auto system = std::make_shared<DirectSystem>();
    
    // 3. Selector: Extracts the state directly
    auto selector = std::make_shared<DirectTargetSelector>();
    
    // 4. Solver: Use Boresight Offset Parallax Logic + P Controller
    auto solver = std::make_shared<LaserAimerSolver>();

    Runtime<std::optional<TargetState>, std::optional<TargetState>> runtime(provider, system, selector, solver);

    runtime.start();
    runtime.runUI();
    runtime.stop();

    std::cout << "Exiting...\n";
    return 0;
}