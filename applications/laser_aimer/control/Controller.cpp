#include "Controller.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <utility>

namespace laser_aimer {
namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr double kDeadbandHystPx = 1.0;
constexpr double kMinScanDs = 1e-3;

double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

std::string toLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return v;
}

}  // namespace

Controller::Controller(ControlConfig cfg) : cfg_(std::move(cfg)) {}

GimbalCommand Controller::update(const TargetMeasurement & meas,
                                 const CameraModel & cam,
                                 const Boresight & bs,
                                 const GimbalState & state) {
  GimbalCommand cmd;
  cmd.timestamp = static_cast<uint32_t>(meas.timestamp_ms);
  cmd.pitch = state.pitch;
  cmd.yaw = state.yaw;

  const auto steady_now = std::chrono::steady_clock::now();
  double dt_s = cfg_.ctrl_dt_nominal_ms / 1000.0;
  if (has_last_update_steady_) {
    double dt_ms = std::chrono::duration<double, std::milli>(steady_now - last_update_tp_).count();
    dt_ms = clamp(dt_ms, cfg_.ctrl_dt_min_ms, cfg_.ctrl_dt_max_ms);
    dt_s = dt_ms / 1000.0;
  }
  last_update_tp_ = steady_now;
  has_last_update_steady_ = true;

  if (!meas.valid || cam.fx <= 0.0 || cam.fy <= 0.0) {
    in_deadband_ = false;
    if (!lost_active_) {
      lost_active_ = true;
      lost_start_ts_ = meas.timestamp_ms;
      reacq_count_ = 0;
    }

    double base_pitch = has_last_ ? last_pitch_ : state.pitch;
    double base_yaw = has_last_ ? last_yaw_ : state.yaw;
    double pitch_cmd = base_pitch;
    double yaw_cmd = base_yaw;
    const double lost_ms = static_cast<double>(meas.timestamp_ms - lost_start_ts_);
    const bool enter_scan = cfg_.scan_enable &&
      (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);

    if (enter_scan) {
      if (!scanning_) {
        double offset = 0.0;
        if (has_last_meas_) {
          double u = cfg_.yaw_sign * (last_uv_.x - static_cast<float>(bs.u_l));
          double v = cfg_.pitch_sign * (last_uv_.y - static_cast<float>(bs.v_l));
          if (std::abs(u) > 1e-6 || std::abs(v) > 1e-6) {
            if (toLower(cfg_.scan_pattern) == "spiral") {
              u /= std::max(1e-3, cfg_.scan_k_yaw);
              v /= std::max(1e-3, cfg_.scan_k_pitch);
            }
            offset = std::atan2(v, u);
          }
        }
        scan_center_pitch_ = base_pitch;
        scan_center_yaw_ = base_yaw;
        scan_phase_ = 0.0;
        scan_phase_offset_ = offset;
        scan_dir_ = 1;
        scanning_ = true;
      }

      if (toLower(cfg_.scan_pattern) == "spiral") {
        double theta = scan_phase_;
        const double spacing = std::max(1e-3, cfg_.scan_spacing_deg);
        const double a = spacing / (2.0 * 3.14159265358979323846);
        double r = a * theta;
        const double r_max = std::max(spacing, cfg_.scan_r_max_deg);
        const double kx = std::max(1e-3, cfg_.scan_k_yaw);
        const double ky = std::max(1e-3, cfg_.scan_k_pitch);
        const double psi = theta + scan_phase_offset_;

        if (cfg_.scan_spiral_return) {
          if (scan_dir_ > 0 && r >= r_max) scan_dir_ = -1;
          else if (scan_dir_ < 0 && theta <= 0.0) scan_dir_ = 1;
        }

        double dx = 0.0;
        double dy = 0.0;
        if (r < r_max) {
          dx = kx * (a * std::cos(psi) - r * std::sin(psi));
          dy = ky * (a * std::sin(psi) + r * std::cos(psi));
        } else {
          r = r_max;
          dx = -kx * r * std::sin(psi);
          dy = ky * r * std::cos(psi);
        }
        double ds = std::max(kMinScanDs, std::sqrt(dx * dx + dy * dy));
        theta += static_cast<double>(scan_dir_) *
          (std::max(1e-3, cfg_.scan_speed_deg_s) / ds) * dt_s;
        if (!cfg_.scan_spiral_return && theta > 1000.0 * 3.14159265358979323846) {
          theta = std::fmod(theta, 2.0 * 3.14159265358979323846);
        }
        theta = std::max(0.0, theta);
        scan_phase_ = theta;
        const double r_eval = std::min(a * theta, r_max);
        pitch_cmd = scan_center_pitch_ + ky * r_eval * std::sin(theta + scan_phase_offset_);
        yaw_cmd = scan_center_yaw_ + kx * r_eval * std::cos(theta + scan_phase_offset_);
      } else {
        scan_phase_ += 2.0 * 3.14159265358979323846 * cfg_.scan_rate_hz * dt_s;
        scan_phase_ = std::fmod(scan_phase_, 2.0 * 3.14159265358979323846);
        pitch_cmd = scan_center_pitch_ + cfg_.scan_radius_deg * std::sin(scan_phase_ + scan_phase_offset_);
        yaw_cmd = scan_center_yaw_ + cfg_.scan_radius_deg * std::cos(scan_phase_ + scan_phase_offset_);
      }
    }

    const double max_step = cfg_.max_angle_rate * dt_s;
    cmd.pitch = static_cast<float>(clamp(pitch_cmd, base_pitch - max_step, base_pitch + max_step));
    cmd.yaw = static_cast<float>(clamp(yaw_cmd, base_yaw - max_step, base_yaw + max_step));
    last_pitch_ = cmd.pitch;
    last_yaw_ = cmd.yaw;
    has_last_ = true;
    return cmd;
  }

  if (lost_active_) {
    reacq_count_++;
    if (reacq_count_ >= std::max(1, cfg_.scan_reacq_confirm_frames)) {
      scanning_ = false;
      lost_active_ = false;
      reacq_count_ = 0;
    }
  }

  const double du = meas.uv.x - bs.u_l;
  const double dv = meas.uv.y - bs.v_l;
  if (in_deadband_) {
    if (std::abs(du) > cfg_.deadband_px + kDeadbandHystPx ||
        std::abs(dv) > cfg_.deadband_px + kDeadbandHystPx) {
      in_deadband_ = false;
    }
  } else if (std::abs(du) < cfg_.deadband_px && std::abs(dv) < cfg_.deadband_px) {
    in_deadband_ = true;
  }

  if (in_deadband_) {
    if (has_last_) {
      cmd.pitch = static_cast<float>(last_pitch_);
      cmd.yaw = static_cast<float>(last_yaw_);
    }
    last_uv_ = meas.uv;
    last_meas_ts_ = meas.timestamp_ms;
    has_last_meas_ = true;
    return cmd;
  }

  double yaw_cmd = state.yaw + cfg_.kp * cfg_.yaw_sign * std::atan(du / cam.fx) * kRadToDeg;
  double pitch_cmd = state.pitch + cfg_.kp * cfg_.pitch_sign * std::atan(dv / cam.fy) * kRadToDeg;

  if (cfg_.use_damping && cfg_.damping_kd > 0.0) {
    bool has_rate = false;
    double yaw_rate = 0.0;
    double pitch_rate = 0.0;
    if (toLower(cfg_.damping_source) == "gimbal") {
      yaw_rate = state.yaw_rate;
      pitch_rate = state.pitch_rate;
      has_rate = true;
    } else if (has_last_meas_) {
      double dt_ms = static_cast<double>(meas.timestamp_ms - last_meas_ts_);
      if (dt_ms > 0.0 && dt_ms <= cfg_.damping_dt_max_ms) {
        const double dt = dt_ms / 1000.0;
        yaw_rate = cfg_.yaw_sign * ((meas.uv.x - last_uv_.x) / dt / cam.fx) * kRadToDeg;
        pitch_rate = cfg_.pitch_sign * ((meas.uv.y - last_uv_.y) / dt / cam.fy) * kRadToDeg;
        has_rate = true;
      }
    }
    if (has_rate) {
      yaw_cmd -= cfg_.damping_kd * yaw_rate;
      pitch_cmd -= cfg_.damping_kd * pitch_rate;
    }
  }

  if (cfg_.use_velocity_ff && has_last_meas_) {
    double dt_ms = static_cast<double>(meas.timestamp_ms - last_meas_ts_);
    if (dt_ms > 0.0 && dt_ms <= cfg_.ff_dt_max_ms) {
      const double dt = dt_ms / 1000.0;
      double yaw_rate = cfg_.yaw_sign * ((meas.uv.x - last_uv_.x) / dt / cam.fx) * kRadToDeg;
      double pitch_rate = cfg_.pitch_sign * ((meas.uv.y - last_uv_.y) / dt / cam.fy) * kRadToDeg;
      yaw_rate = cfg_.ff_alpha * yaw_rate + (1.0 - cfg_.ff_alpha) * last_ff_yaw_rate_;
      pitch_rate = cfg_.ff_alpha * pitch_rate + (1.0 - cfg_.ff_alpha) * last_ff_pitch_rate_;
      last_ff_yaw_rate_ = clamp(yaw_rate, -cfg_.ff_rate_max, cfg_.ff_rate_max);
      last_ff_pitch_rate_ = clamp(pitch_rate, -cfg_.ff_rate_max, cfg_.ff_rate_max);
      cmd.yaw_rate = static_cast<float>(last_ff_yaw_rate_);
      cmd.pitch_rate = static_cast<float>(last_ff_pitch_rate_);
    }
  } else {
    last_ff_yaw_rate_ = 0.0;
    last_ff_pitch_rate_ = 0.0;
  }

  if (has_last_) {
    yaw_cmd = cfg_.lowpass_alpha * yaw_cmd + (1.0 - cfg_.lowpass_alpha) * last_yaw_;
    pitch_cmd = cfg_.lowpass_alpha * pitch_cmd + (1.0 - cfg_.lowpass_alpha) * last_pitch_;
  }

  const double max_step = cfg_.max_angle_rate * dt_s;
  cmd.yaw = static_cast<float>(clamp(yaw_cmd, state.yaw - max_step, state.yaw + max_step));
  cmd.pitch = static_cast<float>(clamp(pitch_cmd, state.pitch - max_step, state.pitch + max_step));
  last_pitch_ = cmd.pitch;
  last_yaw_ = cmd.yaw;
  has_last_ = true;
  last_uv_ = meas.uv;
  last_meas_ts_ = meas.timestamp_ms;
  has_last_meas_ = true;
  return cmd;
}

}  // namespace laser_aimer
