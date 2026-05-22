#include "LaserAimerActuator.hpp"

#include "../../include/core/DebugContext.hpp"
#include "io/GimbalSerial.hpp"
#include "serial/serial.h"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <vector>

namespace laser_aimer {
namespace {

std::string hexBytes(const uint8_t * data, size_t len, size_t max_len = 64) {
  std::ostringstream oss;
  oss << std::hex << std::uppercase << std::setfill('0');
  const size_t shown = std::min(len, max_len);
  for (size_t i = 0; i < shown; ++i) {
    if (i > 0) oss << ' ';
    oss << std::setw(2) << static_cast<int>(data[i]);
  }
  if (shown < len) oss << " ... +" << std::dec << (len - shown) << " bytes";
  return oss.str();
}

std::string formatCommandStatus(const char * prefix, const GimbalCommand & cmd) {
  std::ostringstream oss;
  oss << prefix
      << " pitch=" << std::fixed << std::setprecision(3) << cmd.pitch
      << " yaw=" << cmd.yaw
      << " pr=" << cmd.pitch_rate
      << " yr=" << cmd.yaw_rate
      << " t=" << cmd.timestamp;
  return oss.str();
}

std::string formatStateStatus(const char * prefix, const GimbalState & state) {
  std::ostringstream oss;
  oss << prefix
      << " pitch=" << std::fixed << std::setprecision(3) << state.pitch
      << " yaw=" << state.yaw
      << " pr=" << state.pitch_rate
      << " yr=" << state.yaw_rate
      << " t=" << state.timestamp;
  return oss.str();
}

}  // namespace

LaserAimerActuator::LaserAimerActuator(const LaserAimerConfig & cfg) : cfg_(cfg) {
  if (!cfg_.serial.enabled) return;
  serial::Timeout timeout = serial::Timeout::simpleTimeout(
    static_cast<uint32_t>(std::max(1, cfg_.serial.read_timeout_ms)));
  serial_ = std::make_unique<serial::Serial>(
    cfg_.serial.port,
    static_cast<uint32_t>(cfg_.serial.baud),
    timeout);
  if (!serial_->isOpen()) {
    serial_->open();
  }
  std::cout << "[LaserAimer] Serial opened: " << cfg_.serial.port
            << " baud=" << cfg_.serial.baud << "\n";
}

LaserAimerActuator::~LaserAimerActuator() = default;

bool LaserAimerActuator::shouldLogSerial(int64_t * last_log_ms) const {
  if (!cfg_.serial.debug_io || !last_log_ms) return false;
  const int interval_ms = cfg_.serial.debug_interval_ms;
  const int64_t now = nowMs();
  if (interval_ms <= 0 || *last_log_ms == 0 || now - *last_log_ms >= interval_ms) {
    *last_log_ms = now;
    return true;
  }
  return false;
}

void LaserAimerActuator::send(const aim::Command & cmd) {
  if (!serial_ || !serial_->isOpen()) return;
  GimbalCommand out;
  out.pitch = static_cast<float>(cmd.pitch);
  out.yaw = static_cast<float>(cmd.yaw);
  out.pitch_rate = static_cast<float>(cmd.pitch_vel);
  out.yaw_rate = static_cast<float>(cmd.yaw_vel);
  out.timestamp = static_cast<uint32_t>(nowMs());
  uint8_t packet[gimbal_serial::kTxFrameSize]{};
  gimbal_serial::packGimbalCommand(out, packet);
  const size_t written = serial_->write(packet, gimbal_serial::kTxFrameSize);
  aim::DebugContext::getInstance().setText("Serial TX", formatCommandStatus("TX", out));

  if (shouldLogSerial(&last_tx_log_ms_)) {
    std::cout << "[Serial TX] wrote=" << written
              << " pitch=" << out.pitch
              << " yaw=" << out.yaw
              << " pitch_rate=" << out.pitch_rate
              << " yaw_rate=" << out.yaw_rate
              << " timestamp=" << out.timestamp;
    if (cfg_.serial.debug_hex) {
      std::cout << " bytes=[" << hexBytes(packet, gimbal_serial::kTxFrameSize) << "]";
    }
    std::cout << "\n";
  }
}

std::optional<aim::SelfState> LaserAimerActuator::feedback() {
  if (!serial_ || !serial_->isOpen()) {
    aim::SelfState s;
    s.timestamp = aim::TimePoint(std::chrono::high_resolution_clock::now().time_since_epoch());
    s.valid = true;
    return s;
  }

  const size_t available = serial_->available();
  if (available > 0) {
    std::vector<uint8_t> buf(std::min<size_t>(available, 512));
    size_t n = serial_->read(buf.data(), buf.size());
    const bool log_rx = shouldLogSerial(&last_rx_log_ms_);
    if (log_rx) {
      std::cout << "[Serial RX] read=" << n;
      if (cfg_.serial.debug_hex) {
        std::cout << " bytes=[" << hexBytes(buf.data(), n) << "]";
      }
      std::cout << "\n";
    }

    GimbalState state;
    if (parser_.push(buf.data(), n, &state)) {
      last_state_ = state;
      has_state_ = true;
      aim::DebugContext::getInstance().setText("Serial RX", formatStateStatus("RX", state));
      if (log_rx) {
        std::cout << "[Serial RX parsed]"
                  << " pitch=" << state.pitch
                  << " yaw=" << state.yaw
                  << " pitch_rate=" << state.pitch_rate
                  << " yaw_rate=" << state.yaw_rate
                  << " timestamp=" << state.timestamp << "\n";
      }
    } else if (log_rx) {
      std::cout << "[Serial RX parsed] pending/invalid frame\n";
    }
  }

  if (!has_state_) return std::nullopt;

  aim::SelfState s;
  s.timestamp = aim::TimePoint(std::chrono::high_resolution_clock::now().time_since_epoch());
  s.pitch = last_state_.pitch;
  s.yaw = last_state_.yaw;
  s.pitch_vel = last_state_.pitch_rate;
  s.yaw_vel = last_state_.yaw_rate;
  s.valid = true;
  return s;
}

}  // namespace laser_aimer
