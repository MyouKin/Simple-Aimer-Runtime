#ifndef LASER_AIMER_DETECTOR_FIXED_TARGET_DETECTOR_HPP
#define LASER_AIMER_DETECTOR_FIXED_TARGET_DETECTOR_HPP

#include "../types.hpp"

#include <mutex>
#include <opencv2/core.hpp>
#include <optional>

namespace laser_aimer {

struct FixedTargetConfig {
  double roi_padding_ratio = 0.08;
  int h_min = 0;
  int h_max = 179;
  int s_min = 0;
  int s_max = 255;
  int v_min = 200;
  int v_max = 255;
  int close_kernel = 10;
  double min_area = 50.0;
  double max_area = 3000.0;
  double min_aspect = 1.5;
  double max_aspect = 10.0;
  double max_angle_diff_deg = 15.0;
  double min_dist_ratio = 1.0;
  double max_dist_ratio = 6.0;
};

class FixedTargetDetector {
public:
  explicit FixedTargetDetector(FixedTargetConfig cfg);

  void setConfig(FixedTargetConfig cfg);
  FixedTargetConfig config() const;

  std::optional<FixedTarget> detect(const cv::Mat & bgr,
                                    const std::vector<DroneBox> & drones,
                                    cv::Mat * debug = nullptr) const;

private:
  struct Strip;
  std::optional<FixedTarget> detectInRoi(const cv::Mat & roi,
                                         const cv::Rect & roi_rect,
                                         const FixedTargetConfig & cfg,
                                         cv::Mat * debug) const;
  cv::Rect paddedRoi(const cv::Rect & box,
                     const cv::Size & frame_size,
                     const FixedTargetConfig & cfg) const;

  mutable std::mutex mutex_;
  FixedTargetConfig cfg_;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_DETECTOR_FIXED_TARGET_DETECTOR_HPP
