#ifndef AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
#define AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP

#include "../core/types.hpp"

namespace aim {

/// @brief 系统状态容器 — 消化传感器输入，维护全量系统状态（包括目标跟踪信息与云台自身状态）。
///
/// SystemStateType 由用户定义，建议包含 GimbalState 字段以便 Solver（如 MPC）读取云台姿态。
template <typename InputType, typename SystemStateType>
class System {
public:
    virtual ~System() = default;

    /// @brief 消化传感器/检测输入，更新目标跟踪等内部状态
    virtual void update(const InputType& input) = 0;

    /// @brief 从 Actuator 获得的云台反馈状态写入系统，供 Solver 下一周期使用
    /// 默认空实现 — 如果用户的 SystemStateType 不含 GimbalState 则可忽略
    virtual void updateGimbalState(const GimbalState& /*gimbal*/) {}

    /// @brief 获取当前系统的全量状态（含目标与云台）
    virtual const SystemStateType& getState() const = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
