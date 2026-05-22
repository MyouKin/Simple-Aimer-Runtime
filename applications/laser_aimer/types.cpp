#include "types.hpp"

#include <chrono>

namespace laser_aimer {

int64_t nowMs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

GimbalState toGimbalState(const aim::SelfState & self) {
  GimbalState out;
  out.pitch = static_cast<float>(self.pitch);
  out.yaw = static_cast<float>(self.yaw);
  out.pitch_rate = static_cast<float>(self.pitch_vel);
  out.yaw_rate = static_cast<float>(self.yaw_vel);
  const auto ts = self.timestamp.time_since_epoch();
  out.timestamp = static_cast<uint32_t>(
    std::chrono::duration_cast<std::chrono::milliseconds>(ts).count());
  return out;
}

aim::Command toAimCommand(const GimbalCommand & cmd) {
  aim::Command out;
  out.pitch = cmd.pitch;
  out.yaw = cmd.yaw;
  out.pitch_vel = cmd.pitch_rate;
  out.yaw_vel = cmd.yaw_rate;
  out.is_absolute = true;
  return out;
}

}  // namespace laser_aimer
