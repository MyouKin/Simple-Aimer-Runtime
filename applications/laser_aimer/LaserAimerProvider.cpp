#include "LaserAimerProvider.hpp"

#include "../../include/core/DebugContext.hpp"

#include <algorithm>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace laser_aimer {
namespace {

cv::Point roundedPoint(const cv::Point2f & point) {
  return {cvRound(point.x), cvRound(point.y)};
}

}  // namespace

LaserAimerProvider::LaserAimerProvider(const LaserAimerConfig & cfg)
  : cfg_(cfg),
    camera_(createFrameSource(cfg.camera)),
    drone_detector_(cfg.drone_detector),
    fixed_target_detector_(cfg.fixed_target) {}

void LaserAimerProvider::updateFixedTargetConfig(const FixedTargetConfig & cfg) {
  cfg_.fixed_target = cfg;
  fixed_target_detector_.setConfig(cfg);
}

cv::Mat LaserAimerProvider::applyUndistort(const cv::Mat & frame) {
  if (!cfg_.undistort.enabled || frame.empty()) return frame;

  const cv::Size frame_size = frame.size();
  if (undistort_map1_.empty() || undistort_map_size_ != frame_size) {
    const auto & u = cfg_.undistort;
    if (u.raw_fx <= 0.0 || u.raw_fy <= 0.0 || u.new_fx <= 0.0 || u.new_fy <= 0.0) {
      std::cerr << "[LaserAimer] undistort_enable is set, but camera parameters are invalid; "
                << "using raw frame.\n";
      return frame;
    }

    const cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
      u.raw_fx, 0.0, u.raw_cx,
      0.0, u.raw_fy, u.raw_cy,
      0.0, 0.0, 1.0);
    const cv::Mat dist_coeffs = (cv::Mat_<double>(1, 5) <<
      u.k1, u.k2, u.p1, u.p2, u.k3);
    const cv::Mat new_camera_matrix = (cv::Mat_<double>(3, 3) <<
      u.new_fx, 0.0, u.new_cx,
      0.0, u.new_fy, u.new_cy,
      0.0, 0.0, 1.0);

    cv::initUndistortRectifyMap(
      camera_matrix,
      dist_coeffs,
      cv::Mat::eye(3, 3, CV_64F),
      new_camera_matrix,
      frame_size,
      CV_16SC2,
      undistort_map1_,
      undistort_map2_);
    undistort_map_size_ = frame_size;
    std::cout << "[LaserAimer] Undistort map initialized for "
              << frame_size.width << "x" << frame_size.height << "\n";
  }

  cv::Mat undistorted;
  cv::remap(frame, undistorted, undistort_map1_, undistort_map2_, cv::INTER_LINEAR);
  return undistorted;
}

bool LaserAimerProvider::fetch(LaserAimerInput & out_data) {
  cv::Mat raw_frame;
  SteadyTimePoint timestamp;
  if (!camera_->read(raw_frame, timestamp) || raw_frame.empty()) {
    return false;
  }

  cv::Mat frame = applyUndistort(raw_frame);
  out_data.frame = frame;
  out_data.timestamp = timestamp;
  out_data.timestamp_ms = nowMs();
  out_data.drones = drone_detector_.detect(frame);

  cv::Mat result = frame.clone();
  for (const auto & drone : out_data.drones) {
    cv::rectangle(result, drone.box, cv::Scalar(0, 255, 255), 2);
    const std::string label = drone.label + " " + cv::format("%.2f", drone.confidence);
    const int y = std::max(12, drone.box.y - 4);
    cv::putText(result, label, cv::Point(drone.box.x, y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
  }

  out_data.fixed_target = fixed_target_detector_.detect(frame, out_data.drones, &result);
  out_data.measurement = {};
  out_data.measurement.timestamp_ms = out_data.timestamp_ms;
  if (out_data.fixed_target && out_data.fixed_target->valid) {
    out_data.measurement.valid = true;
    out_data.measurement.uv = out_data.fixed_target->image_point;
    out_data.measurement.confidence = out_data.fixed_target->confidence;
  }

  auto & debug = aim::DebugContext::getInstance();
  if (out_data.measurement.valid) {
    const cv::Point target_point = roundedPoint(out_data.measurement.uv);
    cv::circle(result, target_point, 6, cv::Scalar(0, 255, 255), 2);
    cv::putText(result, "target", target_point + cv::Point(8, -8),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
  }

  cv::Point2f kalman_predict;
  if (debug.getPoint("Kalman Predict", kalman_predict)) {
    const cv::Point kalman_point = roundedPoint(kalman_predict);
    cv::circle(result, kalman_point, 7, cv::Scalar(255, 0, 255), 2);
    cv::drawMarker(result, kalman_point, cv::Scalar(255, 0, 255),
                   cv::MARKER_CROSS, 18, 2);
    cv::putText(result, "kalman", kalman_point + cv::Point(8, 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255), 1);
  }

  const std::string serial_rx = debug.getText("Serial RX");
  const std::string serial_tx = debug.getText("Serial TX");
  int text_y = 24;
  const auto drawStatus = [&](const std::string & text) {
    if (text.empty()) return;
    const cv::Point org(10, text_y);
    cv::putText(result, text, org + cv::Point(1, 1),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 2);
    cv::putText(result, text, org,
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);
    text_y += 24;
  };
  drawStatus(serial_rx);
  drawStatus(serial_tx);

  debug.setImage("Original", frame);
  debug.setImage("Result", result);
  if (out_data.measurement.valid) {
    debug.setTarget2D({out_data.measurement.uv.x, out_data.measurement.uv.y}, true);
  } else {
    debug.setTarget2D({0.0, 0.0}, false);
  }

  return true;
}

}  // namespace laser_aimer
