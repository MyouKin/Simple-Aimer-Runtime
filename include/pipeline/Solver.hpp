#ifndef AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
#define AIM_FRAMEWORK_PIPELINE_SOLVER_HPP

#include "../core/types.hpp"

namespace aim {

/// @brief 控制指令解算器 — 根据目标状态和系统全量状态（含云台状态）解算云台控制指令。
///
/// SystemStateType 由 System 维护，通常包含目标跟踪信息以及云台自身状态（GimbalState）。
/// Solver 通过接收 SystemStateType 可以获得云台当前姿态/角速度，从而支持 MPC 等高级控制。
template <typename SystemStateType>
class Solver {
public:
    virtual ~Solver() = default;

    /// @param target       从 Selector 选出的单一目标状态
    /// @param system_state 系统全量状态（含云台状态），由 System::getState() 提供
    /// @return 云台控制指令
    virtual GimbalCommand solve(const TargetState& target,
                                const SystemStateType& system_state) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
