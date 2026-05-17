#ifndef AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP
#define AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP

#include "../core/types.hpp"

namespace aim {

template <typename SystemStateType>
class Selector {
public:
    virtual ~Selector() = default;

    // 从系统全量状态中提取单一物理目标状态
    virtual TargetState select(const SystemStateType& system_state) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP
