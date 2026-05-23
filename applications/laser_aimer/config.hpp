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

struct UndistortConfig {
  bool enabled = false;
  double raw_fx = 0.0;
  double raw_fy = 0.0;
  double raw_cx = 0.0;
  double raw_cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  double k3 = 0.0;
  double new_fx = 0.0;
  double new_fy = 0.0;
  double new_cx = 0.0;
  double new_cy = 0.0;
};

struct LaserAimerConfig {
  CameraConfig camera;
  UndistortConfig undistort;
  DroneDetectorConfig drone_detector;
  FixedTargetConfig fixed_target;
  ControlConfig control;
  CameraModel camera_model;
  Boresight boresight;
  SerialConfig serial;
  int min_detect_count = 1;
  int max_lost_count = 20;
  bool target_kalman_enable = true;
  double target_kalman_process_noise = 80.0;
  double target_kalman_measurement_noise = 9.0;
  double target_kalman_error = 100.0;
  double target_kalman_max_dt_ms = 100.0;
  double loop_rate_hz = 100.0;
  bool show_debug = true;
};

bool loadConfig(const std::string & path, LaserAimerConfig * out);

}  // namespace laser_aimer

#endif  // LASER_AIMER_CONFIG_HPP
