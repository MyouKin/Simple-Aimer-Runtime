#ifndef LASER_AIMER_TYPES_HPP
#define LASER_AIMER_TYPES_HPP

#include "../../include/pipeline/Command.hpp"
#include "../../include/pipeline/System.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include <vector>

namespace laser_aimer {

using SteadyTimePoint = std::chrono::steady_clock::time_point;

struct TargetMeasurement {
  bool valid = false;
  int64_t timestamp_ms = 0;
  cv::Point2f uv{};
  float confidence = 0.0F;
};

struct GimbalState {
  float pitch = 0.0F;
  float yaw = 0.0F;
  float pitch_rate = 0.0F;
  float yaw_rate = 0.0F;
  uint32_t timestamp = 0;
};

struct GimbalCommand {
  float pitch = 0.0F;
  float yaw = 0.0F;
  float pitch_rate = 0.0F;
  float yaw_rate = 0.0F;
  uint32_t timestamp = 0;
};

struct CameraModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
};

struct Boresight {
  double u_l = 0.0;
  double v_l = 0.0;
};

struct DroneBox {
  bool valid = false;
  cv::Rect box;
  cv::Point2f center{};
  float confidence = 0.0F;
  std::string label;
};

struct FixedTarget {
  bool valid = false;
  cv::Point2f image_point{};
  cv::Rect roi;
  cv::Rect2f local_bounds;
  float confidence = 0.0F;
};

struct LaserAimerInput {
  cv::Mat frame;
  SteadyTimePoint timestamp{};
  int64_t timestamp_ms = 0;
  std::vector<DroneBox> drones;
  std::optional<FixedTarget> fixed_target;
  TargetMeasurement measurement;
};

struct LaserAimerState {
  aim::SelfState self;
  LaserAimerInput last_input;
  TargetMeasurement measurement;
  TargetMeasurement filtered_measurement;
  bool locked = false;
  int detect_count = 0;
  int lost_count = 0;
};

int64_t nowMs();
GimbalState toGimbalState(const aim::SelfState & self);
aim::Command toAimCommand(const GimbalCommand & cmd);

}  // namespace laser_aimer

#endif  // LASER_AIMER_TYPES_HPP
