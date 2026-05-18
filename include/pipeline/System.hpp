#ifndef AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
#define AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP

#include <chrono>

namespace aim {

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

/// @brief 自身平台状态（云台姿态 + 角速度）
///
/// 每个 SystemState **必须**包含此字段（命名为 self）。
/// 由 Actuator 反馈回写，供 Solver（MPC 等）读取。
struct SelfState {
    TimePoint timestamp;
    double yaw = 0.0;
    double pitch = 0.0;
    double yaw_vel = 0.0;
    double pitch_vel = 0.0;
    bool valid = false;
};

/// @brief 系统状态容器
///
/// ## SystemStateType 强制约定
///
/// 每个 SystemStateType **必须**包含：
///   1. SelfState self      — 自身平台运动状态（由 Actuator 反馈回写）
///   2. 用户自定义 TargetState — 对检测目标的建模
///      auto_aim:  EKF 状态 + 状态机
///      laser_aimer: std::optional<TargetData>
///
/// ## 与 Command 的对等地位
///
/// SystemStateType（用户目标建模）和 Solver 的输出（对 GimbalCommand 的选择性填充）
/// 同为**用户级自定义逻辑**。框架提供 SelfState / FinalTargetState / GimbalCommand
/// 作为标准中间格式。
template <typename InputType, typename SystemStateType>
class System {
public:
    virtual ~System() = default;

    virtual void update(const InputType& input) = 0;

    /// @brief 从 Actuator 反馈回写自身状态（写入 state_.self）
    virtual void updateSelfState(const SelfState& self) {}

    virtual const SystemStateType& getState() const = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_SYSTEM_HPP
