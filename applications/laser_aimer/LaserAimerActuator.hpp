#ifndef LASER_AIMER_ACTUATOR_HPP
#define LASER_AIMER_ACTUATOR_HPP

#include "../../include/pipeline/Actuator.hpp"
#include "config.hpp"
#include "io/GimbalSerial.hpp"

#include <cstdint>
#include <memory>
#include <optional>

namespace serial {
class Serial;
}

namespace laser_aimer {

class LaserAimerActuator final : public aim::Actuator {
public:
  explicit LaserAimerActuator(const LaserAimerConfig & cfg);
  ~LaserAimerActuator() override;

  void send(const aim::Command & cmd) override;
  std::optional<aim::SelfState> feedback() override;

private:
  bool shouldLogSerial(int64_t * last_log_ms) const;

  LaserAimerConfig cfg_;
  std::unique_ptr<serial::Serial> serial_;
  gimbal_serial::FrameParser parser_;
  GimbalState last_state_;
  bool has_state_ = false;
  int64_t last_tx_log_ms_ = 0;
  int64_t last_rx_log_ms_ = 0;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_ACTUATOR_HPP
