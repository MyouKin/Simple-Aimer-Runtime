#include "FrameSource.hpp"

#include "MindVisionCamera.hpp"

#include <iostream>
#include <stdexcept>

namespace laser_aimer {
namespace {

class OpenCvFrameSource final : public FrameSource {
public:
  explicit OpenCvFrameSource(const CameraConfig & cfg) : flip_code_(cfg.flip_code) {
    if (!cfg.source.empty()) {
      cap_.open(cfg.source);
    } else {
      cap_.open(cfg.camera_index);
    }
    if (!cap_.isOpened()) {
      throw std::runtime_error("failed to open OpenCV camera/video source");
    }
  }

  bool read(cv::Mat & frame, SteadyTimePoint & timestamp) override {
    if (!cap_.read(frame) || frame.empty()) return false;
    if (flip_code_ != 2) cv::flip(frame, frame, flip_code_);
    timestamp = std::chrono::steady_clock::now();
    return true;
  }

private:
  cv::VideoCapture cap_;
  int flip_code_ = 2;
};

}  // namespace

std::unique_ptr<FrameSource> createFrameSource(const CameraConfig & cfg) {
  if (cfg.backend == "mindvision") {
    return std::make_unique<MindVisionCamera>(cfg);
  }
  if (cfg.backend == "video" || cfg.backend == "image" || cfg.backend == "opencv") {
    return std::make_unique<OpenCvFrameSource>(cfg);
  }
  throw std::runtime_error("unknown camera backend: " + cfg.backend);
}

}  // namespace laser_aimer
