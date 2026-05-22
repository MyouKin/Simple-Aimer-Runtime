#ifndef LASER_AIMER_SYSTEM_HPP
#define LASER_AIMER_SYSTEM_HPP

#include "../../include/pipeline/System.hpp"
#include "config.hpp"
#include "types.hpp"

#include <opencv2/video/tracking.hpp>

namespace laser_aimer {

class LaserAimerSystem final : public aim::System<LaserAimerInput, LaserAimerState> {
public:
  explicit LaserAimerSystem(const LaserAimerConfig & cfg);

  void update(const LaserAimerInput & input) override;
  void updateSelfState(const aim::SelfState & self) override;
  const LaserAimerState & getState() const override { return state_; }

private:
  TargetMeasurement filterMeasurement(const TargetMeasurement & meas);
  void resetTargetFilter();

  LaserAimerConfig cfg_;
  LaserAimerState state_;
  cv::KalmanFilter target_filter_;
  bool target_filter_initialized_ = false;
  int64_t target_filter_last_ts_ = 0;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_SYSTEM_HPP
