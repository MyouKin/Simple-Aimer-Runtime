#include "GimbalSerial.hpp"

#include <algorithm>
#include <cstring>

namespace laser_aimer::gimbal_serial {

bool parseGimbalState(const uint8_t * buf, size_t len, GimbalState * out) {
  if (!buf || !out || len < kRxFrameSize) return false;
  if (buf[0] != kFrameHead || buf[kRxFrameSize - 1] != kFrameTail) return false;
  std::memcpy(&out->pitch, buf + 1, sizeof(float));
  std::memcpy(&out->yaw, buf + 5, sizeof(float));
  std::memcpy(&out->pitch_rate, buf + 9, sizeof(float));
  std::memcpy(&out->yaw_rate, buf + 13, sizeof(float));
  std::memcpy(&out->timestamp, buf + 17, sizeof(uint32_t));
  return true;
}

void packGimbalCommand(const GimbalCommand & cmd, uint8_t out[kTxFrameSize]) {
  out[0] = kFrameHead;
  std::memcpy(out + 1, &cmd.pitch, sizeof(float));
  std::memcpy(out + 5, &cmd.yaw, sizeof(float));
  std::memcpy(out + 9, &cmd.pitch_rate, sizeof(float));
  std::memcpy(out + 13, &cmd.yaw_rate, sizeof(float));
  std::memcpy(out + 17, &cmd.timestamp, sizeof(uint32_t));
  out[21] = kFrameTail;
}

bool FrameParser::push(const uint8_t * data, size_t len, GimbalState * out) {
  if (!data || len == 0) return false;
  buffer_.insert(buffer_.end(), data, data + len);
  bool parsed = false;
  GimbalState latest{};
  while (buffer_.size() >= kRxFrameSize) {
    auto it = std::find(buffer_.begin(), buffer_.end(), kFrameHead);
    if (it == buffer_.end()) {
      buffer_.clear();
      break;
    }
    if (it != buffer_.begin()) buffer_.erase(buffer_.begin(), it);
    if (buffer_.size() < kRxFrameSize) return false;
    if (buffer_[kRxFrameSize - 1] == kFrameTail) {
      bool ok = parseGimbalState(buffer_.data(), kRxFrameSize, &latest);
      buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<long>(kRxFrameSize));
      if (ok) parsed = true;
      continue;
    }
    buffer_.erase(buffer_.begin());
  }
  if (buffer_.size() > kRxFrameSize * 4) {
    buffer_.erase(buffer_.begin(),
                  buffer_.end() - static_cast<long>(kRxFrameSize * 4));
  }
  if (parsed && out) *out = latest;
  return parsed;
}

}  // namespace laser_aimer::gimbal_serial
