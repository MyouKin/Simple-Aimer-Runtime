/// @file GimbalActuator.hpp
/// @brief auto_aim 应用自定义云台执行器 — 封装 GimbalActuator 公共模式
///
/// 实现 Actuator 接口，提供控制模式选择和 transmit()/receive() 虚函数，
/// 子类只需覆写串口收发逻辑。
///
/// 使用方式见 main.cpp 中的 AutoAimGimbal 类。

#ifndef AUTO_AIM__GIMBAL_ACTUATOR_HPP
#define AUTO_AIM__GIMBAL_ACTUATOR_HPP

#include "../../include/pipeline/Actuator.hpp"

namespace aim {

/// @brief 运行时内置云台执行器 — 用户只需覆写串口收发
///
/// 实现 Actuator 接口。提供控制模式选择和 transmit()/receive() 虚函数。
/// is_fine_aiming 标志由应用层 Actuator 按需解析（auto_aim → 开火）。
class GimbalActuator : public Actuator {
public:
  enum class ControlMode { POSITION, VELOCITY, FULL };

  GimbalActuator() = default;
  ~GimbalActuator() override = default;

  void send(const Command & cmd) final;
  std::optional<SelfState> feedback() final;

  void setControlMode(ControlMode mode) { control_mode_ = mode; }

protected:
  virtual bool transmit(const Command & cmd) = 0;
  virtual std::optional<SelfState> receive() { return std::nullopt; }

private:
  ControlMode control_mode_ = ControlMode::VELOCITY;
};

} // namespace aim

#endif