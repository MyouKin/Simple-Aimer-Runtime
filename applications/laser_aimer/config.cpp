#include "config.hpp"

#include <iostream>
#include <opencv2/core.hpp>

namespace laser_aimer {
namespace {

template <typename T>
void readNode(const cv::FileStorage & fs, const std::string & name, T * value) {
  if (!fs[name].empty()) {
    fs[name] >> *value;
  }
}

void readTeam(const cv::FileStorage & fs, FixedTargetConfig * cfg) {
  std::string team;
  readNode(fs, "target_team", &team);
  if (team == "red" || team == "RED" || team == "Red") {
    cfg->target_team = 0;
  } else if (team == "blue" || team == "BLUE" || team == "Blue") {
    cfg->target_team = 1;
  }
  cfg->target_team = cfg->target_team == 0 ? 0 : 1;
}

}  // namespace

bool loadConfig(const std::string & path, LaserAimerConfig * out) {
  if (!out) return false;
  cv::FileStorage fs(path, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cerr << "Failed to open laser aimer config: " << path << "\n";
    return false;
  }

  readNode(fs, "camera_backend", &out->camera.backend);
  readNode(fs, "camera_source", &out->camera.source);
  readNode(fs, "camera_index", &out->camera.camera_index);
  readNode(fs, "exposure_ms", &out->camera.exposure_ms);
  readNode(fs, "analog_gain", &out->camera.analog_gain);
  readNode(fs, "gamma", &out->camera.gamma);
  readNode(fs, "flip_code", &out->camera.flip_code);
  readNode(fs, "camera_timeout_ms", &out->camera.timeout_ms);

  readNode(fs, "undistort_enable", &out->undistort.enabled);
  readNode(fs, "raw_fx", &out->undistort.raw_fx);
  readNode(fs, "raw_fy", &out->undistort.raw_fy);
  readNode(fs, "raw_cx", &out->undistort.raw_cx);
  readNode(fs, "raw_cy", &out->undistort.raw_cy);
  readNode(fs, "dist_k1", &out->undistort.k1);
  readNode(fs, "dist_k2", &out->undistort.k2);
  readNode(fs, "dist_p1", &out->undistort.p1);
  readNode(fs, "dist_p2", &out->undistort.p2);
  readNode(fs, "dist_k3", &out->undistort.k3);
  readNode(fs, "undistort_fx", &out->undistort.new_fx);
  readNode(fs, "undistort_fy", &out->undistort.new_fy);
  readNode(fs, "undistort_cx", &out->undistort.new_cx);
  readNode(fs, "undistort_cy", &out->undistort.new_cy);

  readNode(fs, "drone_backend", &out->drone_detector.backend);
  readNode(fs, "drone_model_path", &out->drone_detector.model_path);
  readNode(fs, "drone_engine_path", &out->drone_detector.engine_path);
  readNode(fs, "drone_input_width", &out->drone_detector.input_width);
  readNode(fs, "drone_input_height", &out->drone_detector.input_height);
  readNode(fs, "drone_conf_threshold", &out->drone_detector.conf_threshold);
  readNode(fs, "drone_nms_threshold", &out->drone_detector.nms_threshold);
  readNode(fs, "drone_top_k", &out->drone_detector.top_k);
  readNode(fs, "drone_max_det", &out->drone_detector.max_det);
  readNode(fs, "drone_device_id", &out->drone_detector.device_id);
  readNode(fs, "drone_max_batch", &out->drone_detector.max_batch);
  readNode(fs, "drone_streams", &out->drone_detector.streams);
  readNode(fs, "drone_auto_streams", &out->drone_detector.auto_streams);
  readNode(fs, "drone_num_classes", &out->drone_detector.num_classes);
  readNode(fs, "drone_apply_sigmoid", &out->drone_detector.apply_sigmoid);
  readNode(fs, "drone_preprocess", &out->drone_detector.preprocess);
  readNode(fs, "drone_output_mode", &out->drone_detector.output_mode);
  readNode(fs, "drone_box_format", &out->drone_detector.box_format);

  readNode(fs, "roi_padding_ratio", &out->fixed_target.roi_padding_ratio);
  readTeam(fs, &out->fixed_target);
  readNode(fs, "red_h_min", &out->fixed_target.red_hsv.h_min);
  readNode(fs, "red_h_max", &out->fixed_target.red_hsv.h_max);
  readNode(fs, "red_s_min", &out->fixed_target.red_hsv.s_min);
  readNode(fs, "red_s_max", &out->fixed_target.red_hsv.s_max);
  readNode(fs, "red_v_min", &out->fixed_target.red_hsv.v_min);
  readNode(fs, "red_v_max", &out->fixed_target.red_hsv.v_max);
  readNode(fs, "blue_h_min", &out->fixed_target.blue_hsv.h_min);
  readNode(fs, "blue_h_max", &out->fixed_target.blue_hsv.h_max);
  readNode(fs, "blue_s_min", &out->fixed_target.blue_hsv.s_min);
  readNode(fs, "blue_s_max", &out->fixed_target.blue_hsv.s_max);
  readNode(fs, "blue_v_min", &out->fixed_target.blue_hsv.v_min);
  readNode(fs, "blue_v_max", &out->fixed_target.blue_hsv.v_max);
  readNode(fs, "h_min", &out->fixed_target.blue_hsv.h_min);
  readNode(fs, "h_max", &out->fixed_target.blue_hsv.h_max);
  readNode(fs, "s_min", &out->fixed_target.blue_hsv.s_min);
  readNode(fs, "s_max", &out->fixed_target.blue_hsv.s_max);
  readNode(fs, "v_min", &out->fixed_target.blue_hsv.v_min);
  readNode(fs, "v_max", &out->fixed_target.blue_hsv.v_max);
  readNode(fs, "open_kernel", &out->fixed_target.open_kernel);
  readNode(fs, "close_kernel", &out->fixed_target.close_kernel);
  readNode(fs, "min_area", &out->fixed_target.min_area);
  readNode(fs, "max_area", &out->fixed_target.max_area);
  readNode(fs, "min_aspect", &out->fixed_target.min_aspect);
  readNode(fs, "max_aspect", &out->fixed_target.max_aspect);

  readNode(fs, "kp", &out->control.kp);
  readNode(fs, "deadband_px", &out->control.deadband_px);
  readNode(fs, "max_angle_rate", &out->control.max_angle_rate);
  readNode(fs, "lowpass_alpha", &out->control.lowpass_alpha);
  readNode(fs, "use_target_kalman", &out->control.use_target_kalman);
  readNode(fs, "target_kalman_accel_noise", &out->control.target_kalman_accel_noise);
  readNode(fs, "target_kalman_meas_noise", &out->control.target_kalman_meas_noise);
  readNode(fs, "target_kalman_reset_dt_ms", &out->control.target_kalman_reset_dt_ms);
  readNode(fs, "yaw_sign", &out->control.yaw_sign);
  readNode(fs, "pitch_sign", &out->control.pitch_sign);
  readNode(fs, "ctrl_dt_nominal_ms", &out->control.ctrl_dt_nominal_ms);
  readNode(fs, "ctrl_dt_min_ms", &out->control.ctrl_dt_min_ms);
  readNode(fs, "ctrl_dt_max_ms", &out->control.ctrl_dt_max_ms);
  readNode(fs, "use_velocity_ff", &out->control.use_velocity_ff);
  readNode(fs, "ff_alpha", &out->control.ff_alpha);
  readNode(fs, "ff_rate_max", &out->control.ff_rate_max);
  readNode(fs, "ff_dt_max_ms", &out->control.ff_dt_max_ms);
  readNode(fs, "use_damping", &out->control.use_damping);
  readNode(fs, "damping_source", &out->control.damping_source);
  readNode(fs, "damping_kd", &out->control.damping_kd);
  readNode(fs, "damping_dt_max_ms", &out->control.damping_dt_max_ms);
  readNode(fs, "scan_enable", &out->control.scan_enable);
  readNode(fs, "scan_radius_deg", &out->control.scan_radius_deg);
  readNode(fs, "scan_rate_hz", &out->control.scan_rate_hz);
  readNode(fs, "scan_pattern", &out->control.scan_pattern);
  readNode(fs, "scan_spacing_deg", &out->control.scan_spacing_deg);
  readNode(fs, "scan_speed_deg_s", &out->control.scan_speed_deg_s);
  readNode(fs, "scan_r_max_deg", &out->control.scan_r_max_deg);
  readNode(fs, "scan_spiral_return", &out->control.scan_spiral_return);
  readNode(fs, "scan_k_yaw", &out->control.scan_k_yaw);
  readNode(fs, "scan_k_pitch", &out->control.scan_k_pitch);
  readNode(fs, "scan_enter_delay_ms", &out->control.scan_enter_delay_ms);
  readNode(fs, "scan_reacq_confirm_frames", &out->control.scan_reacq_confirm_frames);

  readNode(fs, "fx", &out->camera_model.fx);
  readNode(fs, "fy", &out->camera_model.fy);
  readNode(fs, "cx", &out->camera_model.cx);
  readNode(fs, "cy", &out->camera_model.cy);
  readNode(fs, "u_L", &out->boresight.u_l);
  readNode(fs, "v_L", &out->boresight.v_l);

  readNode(fs, "serial_enabled", &out->serial.enabled);
  readNode(fs, "serial_port", &out->serial.port);
  readNode(fs, "serial_baud", &out->serial.baud);
  readNode(fs, "serial_read_timeout_ms", &out->serial.read_timeout_ms);
  readNode(fs, "serial_debug_io", &out->serial.debug_io);
  readNode(fs, "serial_debug_hex", &out->serial.debug_hex);
  readNode(fs, "serial_debug_interval_ms", &out->serial.debug_interval_ms);

  readNode(fs, "min_detect_count", &out->min_detect_count);
  readNode(fs, "max_lost_count", &out->max_lost_count);
  readNode(fs, "loop_rate_hz", &out->loop_rate_hz);
  readNode(fs, "show_debug", &out->show_debug);
  return true;
}

}  // namespace laser_aimer
