#ifndef LASER_AIMER_DETECTOR_DRONE_DETECTOR_HPP
#define LASER_AIMER_DETECTOR_DRONE_DETECTOR_HPP

#include "../types.hpp"

#include <opencv2/dnn.hpp>
#include <memory>
#include <string>
#include <vector>

namespace laser_aimer {

struct DroneDetectorConfig {
  std::string backend = "trt_engine";  // trt_engine | opencv_dnn | full_frame | disabled
  std::string model_path;
  std::string engine_path;
  int input_width = 640;
  int input_height = 640;
  float conf_threshold = 0.25F;
  float nms_threshold = 0.45F;
  int top_k = 1;
  int max_det = 300;
  int device_id = 0;
  int max_batch = 1;
  int streams = 1;
  bool auto_streams = false;
  int num_classes = 1;
  bool apply_sigmoid = false;
  std::string preprocess = "letterbox";
  std::string output_mode = "auto";
  std::string box_format = "cxcywh";
};

class DroneDetector {
public:
  explicit DroneDetector(const DroneDetectorConfig & cfg);
  ~DroneDetector();
  std::vector<DroneBox> detect(const cv::Mat & bgr);

private:
  struct TrtImpl;

  std::vector<DroneBox> fullFrame(const cv::Mat & bgr) const;
  void tryLoadOpenCVDnn();
  void tryLoadTrtEngine();
  std::vector<DroneBox> detectOpenCVDnn(const cv::Mat & bgr);
  std::vector<DroneBox> detectTrtEngine(const cv::Mat & bgr);

  DroneDetectorConfig cfg_;
  cv::dnn::Net net_;
  bool net_loaded_ = false;
  std::unique_ptr<TrtImpl> trt_;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_DETECTOR_DRONE_DETECTOR_HPP
