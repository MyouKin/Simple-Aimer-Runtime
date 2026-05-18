#ifndef AIM_APP_LASER_AIMER_SOLVER_HPP
#define AIM_APP_LASER_AIMER_SOLVER_HPP

#include "../../include/pipeline/Solver.hpp"
#include <opencv2/core.hpp>
#include <chrono>
#include <string>

namespace aim {

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
    int startup_check_frames = 5;
    double startup_home_pitch = 0.0;
    double startup_home_yaw = 0.0;
    int startup_prep_ms = 1000;
    int startup_hold_ms = 200;
    int startup_home_ms = 600;
    int startup_validate_ms = 200;
    int startup_min_state_frames = 1;
};

struct CameraModel {
    double fx = 800.0, fy = 800.0;
    double cx = 640.0, cy = 360.0;
};

struct Boresight {
    double u_L = 320.0, v_L = 240.0;
};

// SystemStateType for laser_aimer: just an optional TargetState (single-target, no tracking history)
using LaserAimerSystemState = std::optional<TargetState>;

class LaserAimerSolver : public Solver<LaserAimerSystemState> {
public:
    LaserAimerSolver();
    explicit LaserAimerSolver(const ControlConfig& cfg,
                              const CameraModel& cam,
                              const Boresight& bs);

    GimbalCommand solve(const TargetState& target,
                        const LaserAimerSystemState& system_state) override;

    static bool loadConfig(const std::string& path,
                           ControlConfig* cfg,
                           CameraModel* cam,
                           Boresight* bs);

    void setGimbalState(const GimbalState& state);
    const GimbalState& gimbalState() const { return gimbal_state_; }
    double deadbandStatus() const { return in_deadband_ ? 1.0 : 0.0; }
    bool isScanning() const { return scanning_; }

private:
    ControlConfig cfg_;
    CameraModel cam_model_;
    Boresight boresight_;
    GimbalState gimbal_state_;

    double last_pitch_ = 0.0, last_yaw_ = 0.0;
    bool has_last_ = false;
    std::chrono::steady_clock::time_point last_update_tp_{};
    bool has_last_update_steady_ = false;
    bool has_last_meas_ = false;
    int64_t last_meas_ts_ = 0;
    cv::Point2f last_uv_{};
    double last_ff_pitch_rate_ = 0.0, last_ff_yaw_rate_ = 0.0;
    bool scanning_ = false;
    double scan_phase_ = 0.0, scan_phase_offset_ = 0.0;
    double scan_center_pitch_ = 0.0, scan_center_yaw_ = 0.0;
    int scan_dir_ = 1;
    bool lost_active_ = false;
    int64_t lost_start_ts_ = 0;
    int reacq_count_ = 0;
    int startup_frames_ = 0;
    bool startup_has_meas_ = false;
    bool startup_prep_started_ = false;
    bool startup_prep_done_ = false;
    std::chrono::steady_clock::time_point startup_prep_tp_{};
    bool in_deadband_ = false;
};

} // namespace aim

#endif