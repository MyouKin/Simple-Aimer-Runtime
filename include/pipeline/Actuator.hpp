#ifndef AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP
#define AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP

#include "../core/types.hpp"
#include <optional>

namespace aim {

/// @brief 执行器抽象 — 将 Solver 解算出的指令发送到硬件，并可回读云台状态形成反馈闭环。
///
/// 典型实现：
///   - 通过串口/CAN 发送 GimbalCommand 到云台驱动板
///   - 从云台驱动板读取当前角度/角速度，封装为 GimbalState 返回
///   - 测试时可使用 NullActuator（丢弃指令）或 SimulatedActuator（模拟云台动力学）
class Actuator {
public:
    virtual ~Actuator() = default;

    /// @brief 发送控制指令到硬件
    virtual void send(const GimbalCommand& cmd) = 0;

    /// @brief 读取云台当前状态反馈
    /// @return 有效的 GimbalState；若无法获取反馈则返回 std::nullopt
    virtual std::optional<GimbalState> feedback() = 0;
};

} // namespace aim

#endif // AIM_FRAMEWORK_PIPELINE_ACTUATOR_HPP