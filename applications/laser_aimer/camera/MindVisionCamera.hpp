#ifndef LASER_AIMER_CAMERA_MINDVISION_CAMERA_HPP
#define LASER_AIMER_CAMERA_MINDVISION_CAMERA_HPP

#include "FrameSource.hpp"

#include <opencv2/core.hpp>

namespace laser_aimer {

class MindVisionCamera final : public FrameSource {
public:
  explicit MindVisionCamera(const CameraConfig & cfg);
  ~MindVisionCamera() override;

  bool read(cv::Mat & frame, SteadyTimePoint & timestamp) override;

private:
  CameraConfig cfg_;
  int handle_ = -1;
  int width_ = 0;
  int height_ = 0;
  bool opened_ = false;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_CAMERA_MINDVISION_CAMERA_HPP
