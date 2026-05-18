#ifndef AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP
#define AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP

#include "FinalTargetState.hpp"

namespace aim {

template <typename SystemStateType>
class Selector {
public:
    virtual ~Selector() = default;
    virtual FinalTargetState select(const SystemStateType& system_state) = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SELECTOR_HPP
