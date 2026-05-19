#ifndef AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP
#define AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP

#include "System.hpp"   // SelfState

namespace aim {

template <typename InputType>
class DataProvider {
public:
    virtual ~DataProvider() = default;

    virtual bool fetch(InputType& out_data) = 0;

    /// @brief 可选：在 fetch() 之前接收 Actuator 反馈（默认空实现）
    virtual void acceptFeedback(const SelfState &) {}
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_DATA_PROVIDER_HPP
