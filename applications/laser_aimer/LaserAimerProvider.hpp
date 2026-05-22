#ifndef LASER_AIMER_PROVIDER_HPP
#define LASER_AIMER_PROVIDER_HPP

#include "../../include/pipeline/DataProvider.hpp"
#include "camera/FrameSource.hpp"
#include "config.hpp"
#include "detector/DroneDetector.hpp"
#include "detector/FixedTargetDetector.hpp"
#include "types.hpp"

#include <memory>

namespace laser_aimer {

class LaserAimerProvider final : public aim::DataProvider<LaserAimerInput> {
public:
  explicit LaserAimerProvider(const LaserAimerConfig & cfg);
  bool fetch(LaserAimerInput & out_data) override;
  void updateFixedTargetConfig(const FixedTargetConfig & cfg);

private:
  LaserAimerConfig cfg_;
  std::unique_ptr<FrameSource> camera_;
  DroneDetector drone_detector_;
  FixedTargetDetector fixed_target_detector_;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_PROVIDER_HPP
