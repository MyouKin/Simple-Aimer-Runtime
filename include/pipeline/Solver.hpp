#ifndef AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
#define AIM_FRAMEWORK_PIPELINE_SOLVER_HPP

#include "../core/types.hpp"

namespace aim {

class Solver {
public:
    virtual ~Solver() = default;

    // 针对单一物理目标解算云台控制指令
    virtual GimbalCommand solve(const TargetState& target) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SOLVER_HPP
