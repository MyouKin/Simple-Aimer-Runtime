#include "FixedTargetDetector.hpp"

#include "../../../include/core/DebugContext.hpp"

#include <algorithm>
#include <cmath>
#include <opencv2/imgproc.hpp>
#include <utility>

namespace laser_aimer {
namespace {

struct PcaInfo {
  cv::Point2f center;
  double angle = 0.0;
  cv::Point2f primary;
  cv::Point2f secondary;
};

PcaInfo performPca(const std::vector<cv::Point> & contour) {
  cv::Mat data(static_cast<int>(contour.size()), 2, CV_64F);
  for (int i = 0; i < data.rows; ++i) {
    data.at<double>(i, 0) = contour[static_cast<size_t>(i)].x;
    data.at<double>(i, 1) = contour[static_cast<size_t>(i)].y;
  }
  cv::PCA pca(data, cv::Mat(), cv::PCA::DATA_AS_ROW);
  PcaInfo out;
  out.center = {
    static_cast<float>(pca.mean.at<double>(0, 0)),
    static_cast<float>(pca.mean.at<double>(0, 1))};
  out.primary = {
    static_cast<float>(pca.eigenvectors.at<double>(0, 0)),
    static_cast<float>(pca.eigenvectors.at<double>(0, 1))};
  out.secondary = {
    static_cast<float>(pca.eigenvectors.at<double>(1, 0)),
    static_cast<float>(pca.eigenvectors.at<double>(1, 1))};
  out.angle = std::atan2(out.primary.y, out.primary.x);
  return out;
}

double normPoint(const cv::Point2f & p) {
  return std::sqrt(p.x * p.x + p.y * p.y);
}

void publishRoiDebugImages(const cv::Mat & roi,
                           const cv::Mat & hsv_view,
                           const cv::Mat & mask,
                           const cv::Mat & closed) {
  cv::Mat mask_bgr;
  cv::Mat closed_bgr;
  cv::cvtColor(mask, mask_bgr, cv::COLOR_GRAY2BGR);
  cv::cvtColor(closed, closed_bgr, cv::COLOR_GRAY2BGR);

  cv::Mat roi_hsv_mask;
  cv::hconcat(std::vector<cv::Mat>{roi, hsv_view, mask_bgr}, roi_hsv_mask);
  auto & debug = aim::DebugContext::getInstance();
  debug.setImage("ROI HSV Mask", roi_hsv_mask);
  debug.setImage("ROI Closed", closed_bgr);
}

}  // namespace

struct FixedTargetDetector::Strip {
  std::vector<cv::Point> contour;
  cv::RotatedRect rect;
  cv::Point2f center;
  cv::Point2f secondary;
  double area = 0.0;
  double length = 0.0;
  double angle = 0.0;
};

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
  cv::Mat hsv_view;
  cv::cvtColor(hsv, hsv_view, cv::COLOR_HSV2BGR);

  cv::Mat mask;
  if (cfg.h_min <= cfg.h_max) {
    cv::inRange(hsv,
                cv::Scalar(cfg.h_min, cfg.s_min, cfg.v_min),
                cv::Scalar(cfg.h_max, cfg.s_max, cfg.v_max),
                mask);
  } else {
    cv::Mat mask1;
    cv::Mat mask2;
    cv::inRange(hsv, cv::Scalar(0, cfg.s_min, cfg.v_min),
                cv::Scalar(cfg.h_max, cfg.s_max, cfg.v_max), mask1);
    cv::inRange(hsv, cv::Scalar(cfg.h_min, cfg.s_min, cfg.v_min),
                cv::Scalar(179, cfg.s_max, cfg.v_max), mask2);
    cv::bitwise_or(mask1, mask2, mask);
  }

  int k = std::max(1, cfg.close_kernel);
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k, k));
  cv::Mat closed;
  cv::morphologyEx(mask, closed, cv::MORPH_CLOSE, kernel);

  publishRoiDebugImages(roi, hsv_view, mask, closed);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<Strip> strips;
  for (const auto & contour : contours) {
    if (contour.size() < 5) continue;
    const double area = cv::contourArea(contour);
    if (area < cfg.min_area || area > cfg.max_area) continue;

    cv::RotatedRect rect = cv::minAreaRect(contour);
    double w = rect.size.width;
    double h = rect.size.height;
    if (w <= 1e-3 || h <= 1e-3) continue;
    double length = std::max(w, h);
    double aspect = length / std::min(w, h);
    if (aspect < cfg.min_aspect || aspect > cfg.max_aspect) continue;

    PcaInfo pca = performPca(contour);
    strips.push_back({contour, rect, pca.center, pca.secondary, area, length, pca.angle});

    if (debug) {
      cv::Point2f pts[4];
      rect.points(pts);
      for (auto & pt : pts) pt += cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
      for (int i = 0; i < 4; ++i) cv::line(*debug, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 1);
    }
  }

  std::optional<FixedTarget> best;
  double best_score = -1.0;
  for (size_t i = 0; i < strips.size(); ++i) {
    for (size_t j = i + 1; j < strips.size(); ++j) {
      const auto & a = strips[i];
      const auto & b = strips[j];
      double angle_diff = std::abs(a.angle - b.angle) * 180.0 / CV_PI;
      if (angle_diff > 90.0) angle_diff = 180.0 - angle_diff;
      if (angle_diff > cfg.max_angle_diff_deg) continue;

      const cv::Point2f delta = a.center - b.center;
      const double center_dist = normPoint(delta);
      const double avg_length = (a.length + b.length) * 0.5;
      if (avg_length <= 1e-3) continue;
      const double dist_ratio = center_dist / avg_length;
      if (dist_ratio < cfg.min_dist_ratio || dist_ratio > cfg.max_dist_ratio) continue;

      const double score = (a.area + b.area) / (1.0 + angle_diff) -
        10.0 * std::abs(dist_ratio - (cfg.min_dist_ratio + cfg.max_dist_ratio) * 0.5);
      if (score <= best_score) continue;

      cv::Point2f local = (a.center + b.center) * 0.5F;
      FixedTarget target;
      target.valid = true;
      target.image_point = local + cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
      target.roi = roi_rect;
      cv::Rect2f a_bounds = a.rect.boundingRect2f();
      cv::Rect2f b_bounds = b.rect.boundingRect2f();
      const float x1 = std::min(a_bounds.x, b_bounds.x);
      const float y1 = std::min(a_bounds.y, b_bounds.y);
      const float x2 = std::max(a_bounds.x + a_bounds.width, b_bounds.x + b_bounds.width);
      const float y2 = std::max(a_bounds.y + a_bounds.height, b_bounds.y + b_bounds.height);
      target.local_bounds = cv::Rect2f(x1, y1, x2 - x1, y2 - y1);
      target.confidence = static_cast<float>(std::max(0.0, score));
      best = target;
      best_score = score;

      if (debug) {
        cv::Point2f pa = a.center + cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
        cv::Point2f pb = b.center + cv::Point2f(static_cast<float>(roi_rect.x), static_cast<float>(roi_rect.y));
        cv::line(*debug, pa, pb, cv::Scalar(255, 0, 0), 2);
      }
    }
  }

  return best;
}

}  // namespace laser_aimer
