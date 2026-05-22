#ifndef LASER_AIMER_IO_GIMBAL_SERIAL_HPP
#define LASER_AIMER_IO_GIMBAL_SERIAL_HPP

#include "../types.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace laser_aimer::gimbal_serial {

constexpr uint8_t kFrameHead = 0xCD;
constexpr uint8_t kFrameTail = 0xDC;
constexpr size_t kRxFrameSize = 22;
constexpr size_t kTxFrameSize = 22;

bool parseGimbalState(const uint8_t * buf, size_t len, GimbalState * out);
void packGimbalCommand(const GimbalCommand & cmd, uint8_t out[kTxFrameSize]);

class FrameParser {
public:
  bool push(const uint8_t * data, size_t len, GimbalState * out);

private:
  std::vector<uint8_t> buffer_;
};

}  // namespace laser_aimer::gimbal_serial

#endif  // LASER_AIMER_IO_GIMBAL_SERIAL_HPP
