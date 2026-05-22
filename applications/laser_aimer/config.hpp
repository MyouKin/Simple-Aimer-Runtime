#ifndef LASER_AIMER_CONFIG_HPP
#define LASER_AIMER_CONFIG_HPP

#include "control/Controller.hpp"
#include "detector/DroneDetector.hpp"
#include "detector/FixedTargetDetector.hpp"
#include "types.hpp"

#include <string>

namespace laser_aimer {

struct CameraConfig {
  std::string backend = "mindvision";  // mindvision | video | image
  std::string source;
  int camera_index = 0;
  double exposure_ms = 2.0;
  double analog_gain = 1.0;
  double gamma = 1.0;
  int flip_code = 2;  // 0 vertical, 1 horizontal, -1 both, 2 disabled
  int timeout_ms = 100;
};

struct SerialConfig {
  bool enabled = true;
  std::string port = "COM1";
  int baud = 115200;
  int read_timeout_ms = 2;
  bool debug_io = false;
  bool debug_hex = true;
  int debug_interval_ms = 100;
};

struct LaserAimerConfig {
  CameraConfig camera;
  DroneDetectorConfig drone_detector;
  FixedTargetConfig fixed_target;
  ControlConfig control;
  CameraModel camera_model;
  Boresight boresight;
  SerialConfig serial;
  int min_detect_count = 1;
  int max_lost_count = 20;
  double loop_rate_hz = 100.0;
  bool show_debug = true;
};

bool loadConfig(const std::string & path, LaserAimerConfig * out);

}  // namespace laser_aimer

#endif  // LASER_AIMER_CONFIG_HPP
