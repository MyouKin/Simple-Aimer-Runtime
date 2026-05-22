#ifndef LASER_AIMER_CONTROL_CONTROLLER_HPP
#define LASER_AIMER_CONTROL_CONTROLLER_HPP

#include "../types.hpp"

#include <chrono>
#include <opencv2/core.hpp>
#include <string>

namespace laser_aimer {

struct ControlConfig {
  double kp = 1.2;
  double deadband_px = 1.0;
  double max_angle_rate = 180.0;
  double lowpass_alpha = 0.45;
  double yaw_sign = -1.0;
  double pitch_sign = -1.0;
  double ctrl_dt_nominal_ms = 10.0;
  double ctrl_dt_min_ms = 1.0;
  double ctrl_dt_max_ms = 100.0;
  bool use_velocity_ff = true;
  double ff_alpha = 0.25;
  double ff_rate_max = 150.0;
  double ff_dt_max_ms = 80.0;
  bool use_damping = true;
  std::string damping_source = "meas";
  double damping_kd = 0.02;
  double damping_dt_max_ms = 120.0;
  bool scan_enable = true;
  double scan_radius_deg = 12.0;
  double scan_rate_hz = 0.2;
  std::string scan_pattern = "spiral";
  double scan_spacing_deg = 6.05;
  double scan_speed_deg_s = 25.0;
  double scan_r_max_deg = 15.0;
  bool scan_spiral_return = false;
  double scan_k_yaw = 2.5;
  double scan_k_pitch = 0.75;
  double scan_enter_delay_ms = 180.0;
  int scan_reacq_confirm_frames = 2;
};

class Controller {
public:
  explicit Controller(ControlConfig cfg = {});

  GimbalCommand update(const TargetMeasurement & meas,
                       const CameraModel & cam,
                       const Boresight & bs,
                       const GimbalState & state);

private:
  ControlConfig cfg_;
  double last_pitch_ = 0.0;
  double last_yaw_ = 0.0;
  bool has_last_ = false;
  bool has_last_meas_ = false;
  int64_t last_meas_ts_ = 0;
  cv::Point2f last_uv_{};
  double last_ff_pitch_rate_ = 0.0;
  double last_ff_yaw_rate_ = 0.0;
  bool has_last_update_steady_ = false;
  std::chrono::steady_clock::time_point last_update_tp_{};
  bool scanning_ = false;
  bool lost_active_ = false;
  int64_t lost_start_ts_ = 0;
  int reacq_count_ = 0;
  double scan_phase_ = 0.0;
  double scan_phase_offset_ = 0.0;
  double scan_center_pitch_ = 0.0;
  double scan_center_yaw_ = 0.0;
  int scan_dir_ = 1;
  bool in_deadband_ = false;
};

}  // namespace laser_aimer

#endif  // LASER_AIMER_CONTROL_CONTROLLER_HPP
