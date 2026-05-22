#include "LaserAimerSystem.hpp"

namespace laser_aimer {

LaserAimerSystem::LaserAimerSystem(const LaserAimerConfig & cfg) : cfg_(cfg) {}

void LaserAimerSystem::update(const LaserAimerInput & input) {
  state_.last_input = input;
  state_.measurement = input.measurement;

  if (input.measurement.valid) {
    state_.detect_count++;
    state_.lost_count = 0;
    if (!state_.filtered_measurement.valid) {
      state_.filtered_measurement = input.measurement;
    } else {
      constexpr float alpha = 0.65F;
      state_.filtered_measurement.valid = true;
      state_.filtered_measurement.timestamp_ms = input.measurement.timestamp_ms;
      state_.filtered_measurement.uv =
        alpha * input.measurement.uv + (1.0F - alpha) * state_.filtered_measurement.uv;
      state_.filtered_measurement.confidence = input.measurement.confidence;
    }
    state_.locked = state_.detect_count >= cfg_.min_detect_count;
    return;
  }

  state_.detect_count = 0;
  state_.lost_count++;
  if (state_.lost_count > cfg_.max_lost_count) {
    state_.locked = false;
    state_.filtered_measurement = {};
  } else {
    state_.filtered_measurement.valid = false;
    state_.filtered_measurement.timestamp_ms = input.timestamp_ms;
  }
}

void LaserAimerSystem::updateSelfState(const aim::SelfState & self) {
  state_.self = self;
}

}  // namespace laser_aimer
