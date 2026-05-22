#include "LaserAimerProvider.hpp"

#include "../../include/core/DebugContext.hpp"

#include <opencv2/imgproc.hpp>

namespace laser_aimer {

LaserAimerProvider::LaserAimerProvider(const LaserAimerConfig & cfg)
  : cfg_(cfg),
    camera_(createFrameSource(cfg.camera)),
    drone_detector_(cfg.drone_detector),
    fixed_target_detector_(cfg.fixed_target) {}

void LaserAimerProvider::updateFixedTargetConfig(const FixedTargetConfig & cfg) {
  cfg_.fixed_target = cfg;
  fixed_target_detector_.setConfig(cfg);
}

bool LaserAimerProvider::fetch(LaserAimerInput & out_data) {
  cv::Mat frame;
  SteadyTimePoint timestamp;
  if (!camera_->read(frame, timestamp) || frame.empty()) {
    return false;
  }

  out_data.frame = frame;
  out_data.timestamp = timestamp;
  out_data.timestamp_ms = nowMs();
  out_data.drones = drone_detector_.detect(frame);

  cv::Mat debug = frame.clone();
  for (const auto & drone : out_data.drones) {
    cv::rectangle(debug, drone.box, cv::Scalar(0, 255, 255), 2);
    const std::string label = drone.label + " " + cv::format("%.2f", drone.confidence);
    const int y = std::max(12, drone.box.y - 4);
    cv::putText(debug, label, cv::Point(drone.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
  }

  out_data.fixed_target = fixed_target_detector_.detect(frame, out_data.drones, &debug);
  out_data.measurement = {};
  out_data.measurement.timestamp_ms = out_data.timestamp_ms;
  if (out_data.fixed_target && out_data.fixed_target->valid) {
    out_data.measurement.valid = true;
    out_data.measurement.uv = out_data.fixed_target->image_point;
    out_data.measurement.confidence = out_data.fixed_target->confidence;
  }

  aim::DebugContext::getInstance().setImage("LaserAimer", debug);
  if (out_data.measurement.valid) {
    aim::DebugContext::getInstance().setTarget2D(
      {out_data.measurement.uv.x, out_data.measurement.uv.y}, true);
  } else {
    aim::DebugContext::getInstance().setTarget2D({0.0, 0.0}, false);
  }

  return true;
}

}  // namespace laser_aimer
