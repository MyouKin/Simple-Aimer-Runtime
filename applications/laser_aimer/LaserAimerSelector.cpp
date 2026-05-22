#include "LaserAimerSelector.hpp"

namespace laser_aimer {

aim::FinalTargetState LaserAimerSelector::select(const LaserAimerState & system_state) {
  aim::FinalTargetState target;
  target.timestamp = std::chrono::high_resolution_clock::now();
  target.frame_id = "camera";
  target.valid = system_state.locked && system_state.filtered_measurement.valid;
  if (target.valid) {
    target.has_image_point = true;
    target.image_point = {
      system_state.filtered_measurement.uv.x,
      system_state.filtered_measurement.uv.y};
  }
  return target;
}

}  // namespace laser_aimer
