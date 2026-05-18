#ifndef AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP
#define AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP

#include "Command.hpp"
#include "System.hpp"   // SelfState
#include <optional>

namespace aim {

/// @brief 执行器抽象 — 发送 Command 到硬件，回读 SelfState
///
/// Command 是唯一控制指令类型。Solver 选择性填充字段，
/// Actuator 按需解析（如将 is_fine_aiming 映射为开火）。
class Actuator {
public:
    virtual ~Actuator() = default;
    virtual void send(const Command& cmd) = 0;
    virtual std::optional<SelfState> feedback() = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP