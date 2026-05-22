#ifndef LASER_AIMER_CAMERA_FRAME_SOURCE_HPP
#define LASER_AIMER_CAMERA_FRAME_SOURCE_HPP

#include "../config.hpp"
#include "../types.hpp"

#include <memory>
#include <opencv2/opencv.hpp>

namespace laser_aimer {

class FrameSource {
public:
  virtual ~FrameSource() = default;
  virtual bool read(cv::Mat & frame, SteadyTimePoint & timestamp) = 0;
};

std::unique_ptr<FrameSource> createFrameSource(const CameraConfig & cfg);

}  // namespace laser_aimer

#endif  // LASER_AIMER_CAMERA_FRAME_SOURCE_HPP
