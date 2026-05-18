#ifndef AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
#define AIM_FRAMEWORK_PIPELINE_SOLVER_HPP

#include "FinalTargetState.hpp"
#include "Command.hpp"
#include "System.hpp"   // SelfState

namespace aim {

/// @brief 控制指令解算器 — 从目标状态和系统全量状态解算 Command
template <typename SystemStateType>
class Solver {
public:
    virtual ~Solver() = default;
    virtual Command solve(const FinalTargetState& target,
                          const SystemStateType& system_state) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
