/// @file GimbalActuator.cpp — 运行时内置云台执行器实现
#include "GimbalActuator.hpp"

namespace aim {

void GimbalActuator::send(const Command & cmd) {
  Command filtered = cmd;

  // 根据控制模式，清零不需要的字段
  switch (control_mode_) {
    case ControlMode::POSITION:
      filtered.yaw_vel = 0; filtered.pitch_vel = 0;
      filtered.yaw_acc = 0; filtered.pitch_acc = 0;
      break;
    case ControlMode::VELOCITY:
      filtered.yaw = 0; filtered.pitch = 0;
      filtered.yaw_acc = 0; filtered.pitch_acc = 0;
      break;
    case ControlMode::FULL:
      // 全部透传，不做过滤
      break;
  }

  transmit(filtered);
}

std::optional<SelfState> GimbalActuator::feedback() {
  return receive();
}

} // namespace aim