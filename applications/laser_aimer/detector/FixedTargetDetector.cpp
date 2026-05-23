#include "FixedTargetDetector.hpp"

#include "../../../include/core/DebugContext.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace laser_aimer {
namespace {

void publishRoiDebugImages(const cv::Mat & mask,
                           const cv::Mat & closed) {
  cv::Mat closed_bgr;
  cv::cvtColor(closed, closed_bgr, cv::COLOR_GRAY2BGR);

  auto & debug = aim::DebugContext::getInstance();
  debug.setImage("ROI HSV Mask", mask);
  debug.setImage("ROI Closed", closed_bgr);
}

const HsvThreshold & activeHsv(const FixedTargetConfig & cfg) {
  return cfg.target_team == 0 ? cfg.red_hsv : cfg.blue_hsv;
}

}  // namespace

FixedTargetDetector::FixedTargetDetector(FixedTargetConfig cfg) : cfg_(std::move(cfg)) {}

void FixedTargetDetector::setConfig(FixedTargetConfig cfg) {
  std::lock_guard<std::mutex> lock(mutex_);
  cfg_ = std::move(cfg);
}

FixedTargetConfig FixedTargetDetector::config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return cfg_;
}

std::optional<FixedTarget> FixedTargetDetector::detect(
  const cv::Mat & bgr, const std::vector<DroneBox> & drones, cv::Mat * debug) const {
  if (bgr.empty()) return std::nullopt;
  const FixedTargetConfig cfg = config();
  if (debug && debug->empty()) *debug = bgr.clone();

  std::optional<FixedTarget> best;
  for (const auto & drone : drones) {
    if (!drone.valid || drone.box.area() <= 0) continue;
    cv::Rect roi_rect = paddedRoi(drone.box, bgr.size(), cfg);
    if (roi_rect.area() <= 0) continue;
    auto candidate = detectInRoi(bgr(roi_rect), roi_rect, cfg, debug);
    if (!candidate) continue;
    if (!best || candidate->confidence > best->confidence) best = candidate;
  }

  if (debug && best && best->valid) {
    cv::circle(*debug, best->image_point, 5, cv::Scalar(0, 0, 255), -1);
    cv::rectangle(*debug, best->roi, cv::Scalar(255, 0, 0), 2);
  }
  return best;
}

cv::Rect FixedTargetDetector::paddedRoi(const cv::Rect & box,
                                        const cv::Size & frame_size,
                                        const FixedTargetConfig & cfg) const {
  int pad_x = static_cast<int>(box.width * cfg.roi_padding_ratio);
  int pad_y = static_cast<int>(box.height * cfg.roi_padding_ratio);
  cv::Rect roi(box.x - pad_x, box.y - pad_y, box.width + 2 * pad_x, box.height + 2 * pad_y);
  return roi & cv::Rect(0, 0, frame_size.width, frame_size.height);
}

std::optional<FixedTarget> FixedTargetDetector::detectInRoi(
  const cv::Mat & roi,
  const cv::Rect & roi_rect,
  const FixedTargetConfig & cfg,
  cv::Mat * debug) const {
  cv::Mat hsv;
  cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  const HsvThreshold & hsv_cfg = activeHsv(cfg);
  if (hsv_cfg.h_min <= hsv_cfg.h_max) {
    cv::inRange(hsv,
                cv::Scalar(hsv_cfg.h_min, hsv_cfg.s_min, hsv_cfg.v_min),
                cv::Scalar(hsv_cfg.h_max, hsv_cfg.s_max, hsv_cfg.v_max),
                mask);
  } else {
    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(0, hsv_cfg.s_min, hsv_cfg.v_min),
                cv::Scalar(hsv_cfg.h_max, hsv_cfg.s_max, hsv_cfg.v_max), mask1);
    cv::inRange(hsv, cv::Scalar(hsv_cfg.h_min, hsv_cfg.s_min, hsv_cfg.v_min),
                cv::Scalar(179, hsv_cfg.s_max, hsv_cfg.v_max), mask2);
    cv::bitwise_or(mask1, mask2, mask);
  }

  int open_k = std::max(1, cfg.open_kernel);
  cv::Mat open_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(open_k, open_k));
  cv::Mat opened;
  cv::morphologyEx(mask, opened, cv::MORPH_OPEN, open_kernel);

  int close_k = std::max(1, cfg.close_kernel);
  cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(close_k, close_k));
  cv::Mat closed;
  cv::morphologyEx(opened, closed, cv::MORPH_CLOSE, close_kernel);

  publishRoiDebugImages(mask, closed);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::optional<FixedTarget> best;
  double best_score = -1.0;
  for (const auto & contour : contours) {
    const double area = cv::contourArea(contour);
    if (area < cfg.min_area || area > cfg.max_area) continue;

    const cv::Rect bounds = cv::boundingRect(contour);
    double w = static_cast<double>(bounds.width);
    double h = static_cast<double>(bounds.height);
    if (w <= 1e-3 || h <= 1e-3) continue;
    double aspect = w / h;
    if (aspect < cfg.min_aspect || aspect > cfg.max_aspect) continue;

    const cv::Moments moments = cv::moments(contour);
    if (std::abs(moments.m00) <= 1e-6) continue;
    const cv::Point2f local(
      static_cast<float>(moments.m10 / moments.m00),
      static_cast<float>(moments.m01 / moments.m00));

    const double aspect_score = 1.0 / (1.0 + std::abs(aspect - 1.0));
    const double score = area * aspect_score;
    if (score <= best_score) continue;

    FixedTarget target;
    target.valid = true;
    target.image_point = local + cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
    target.roi = roi_rect;
    target.local_bounds = cv::Rect2f(
      static_cast<float>(bounds.x),
      static_cast<float>(bounds.y),
      static_cast<float>(bounds.width),
      static_cast<float>(bounds.height));
    target.confidence = static_cast<float>(score);
    best = target;
    best_score = score;

    if (debug) {
      std::vector<std::vector<cv::Point>> shifted{contour};
      for (auto & point : shifted.front()) {
        point.x += roi_rect.x;
        point.y += roi_rect.y;
      }
      cv::drawContours(*debug, shifted, 0, cv::Scalar(0, 255, 0), 2);
      cv::rectangle(*debug, bounds + roi_rect.tl(), cv::Scalar(255, 0, 0), 2);
      cv::circle(*debug, target.image_point, 4, cv::Scalar(0, 0, 255), -1);
    }
  }

  return best;
}

}  // namespace laser_aimer
