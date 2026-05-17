#include "LaserAimerSolver.hpp"
#include "../../include/core/Registry.hpp"
#include "../../include/core/DebugContext.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>

namespace aim {
namespace {
constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDeadbandHystPx = 1.0;
constexpr double kMinScanDs = 1e-3;
double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }
std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return s;
}
}

LaserAimerSolver::LaserAimerSolver() {
    auto& reg = ParameterRegistry::getInstance();
    reg.registerFloat("Camera fx", &cam_model_.fx, 200.0, 3000.0);
    reg.registerFloat("Camera fy", &cam_model_.fy, 200.0, 3000.0);
    reg.registerFloat("Boresight u_L", &boresight_.u_L, 0.0, 1280.0);
    reg.registerFloat("Boresight v_L", &boresight_.v_L, 0.0, 720.0);
    reg.registerFloat("Control Kp", &cfg_.kp, 0.0, 5.0);
    reg.registerFloat("Deadband px", &cfg_.deadband_px, 0.0, 50.0);
    reg.registerFloat("Damping Kd", &cfg_.damping_kd, 0.0, 0.5);
    reg.registerFloat("Max Angle Rate", &cfg_.max_angle_rate, 1.0, 500.0);
    reg.registerFloat("Scan Radius deg", &cfg_.scan_radius_deg, 0.5, 40.0);
    reg.registerFloat("Scan Rate Hz", &cfg_.scan_rate_hz, 0.05, 2.0);
}
LaserAimerSolver::LaserAimerSolver(const ControlConfig& cfg, const CameraModel& cam, const Boresight& bs)
    : cfg_(cfg), cam_model_(cam), boresight_(bs) {}
void LaserAimerSolver::setGimbalState(const GimbalState& state) { gimbal_state_ = state; }

GimbalCommand LaserAimerSolver::solve(const TargetState& target) {
    GimbalCommand cmd; cmd.yaw_vel = 0.0; cmd.pitch_vel = 0.0;
    const auto steady_now = std::chrono::steady_clock::now();
    double dt_s = cfg_.ctrl_dt_nominal_ms / 1000.0;
    if (has_last_update_steady_) {
        double dt_ms = std::chrono::duration<double, std::milli>(steady_now - last_update_tp_).count();
        dt_ms = clamp(dt_ms, cfg_.ctrl_dt_min_ms, cfg_.ctrl_dt_max_ms);
        dt_s = dt_ms / 1000.0;
    }
    last_update_tp_ = steady_now; has_last_update_steady_ = true;

    bool meas_valid = target.valid && target.has_image_point;
    cv::Point2f uv(target.image_point.x(), target.image_point.y());
    int64_t meas_ts = target.timestamp.time_since_epoch().count();

    if (cfg_.startup_prep_ms > 0 && !startup_prep_done_) {
        if (!startup_prep_started_) { startup_prep_started_ = true; startup_prep_tp_ = steady_now; std::cout << "DBG startup_prep begin\n"; }
        int hold_ms = std::max(0, cfg_.startup_hold_ms), home_ms = std::max(0, cfg_.startup_home_ms), validate_ms = std::max(0, cfg_.startup_validate_ms);
        int64_t elapsed = static_cast<int64_t>(std::chrono::duration<double, std::milli>(steady_now - startup_prep_tp_).count());
        if (elapsed < hold_ms + validate_ms + home_ms) {
            double pc = gimbal_state_.pitch, yc = gimbal_state_.yaw;
            if (elapsed >= hold_ms + validate_ms) {
                double ms = cfg_.max_angle_rate * dt_s;
                pc = clamp(cfg_.startup_home_pitch, gimbal_state_.pitch - ms, gimbal_state_.pitch + ms);
                yc = clamp(cfg_.startup_home_yaw, gimbal_state_.yaw - ms, gimbal_state_.yaw + ms);
            }
            cmd.yaw_vel = yc; cmd.pitch_vel = pc; cmd.is_absolute = true;
            last_pitch_ = pc; last_yaw_ = yc; has_last_ = true;
            DebugContext::getInstance().setTarget2D(Vec2::Zero(), false);
            return cmd;
        }
        startup_prep_done_ = true; std::cout << "DBG startup_prep done\n";
    }
    if (cfg_.startup_prep_ms <= 0 && startup_frames_ < std::max(0, cfg_.startup_check_frames)) {
        startup_frames_++; if (meas_valid) startup_has_meas_ = true;
        if (!startup_has_meas_ && startup_frames_ >= cfg_.startup_check_frames) {
            double ms = cfg_.max_angle_rate * dt_s;
            cmd.yaw_vel = clamp(cfg_.startup_home_yaw, gimbal_state_.yaw - ms, gimbal_state_.yaw + ms);
            cmd.pitch_vel = clamp(cfg_.startup_home_pitch, gimbal_state_.pitch - ms, gimbal_state_.pitch + ms);
            cmd.is_absolute = true; last_pitch_ = cmd.pitch_vel; last_yaw_ = cmd.yaw_vel; has_last_ = true;
            DebugContext::getInstance().setTarget2D(Vec2::Zero(), false); return cmd;
        }
    }
    if (!meas_valid || cam_model_.fx <= 0.0 || cam_model_.fy <= 0.0) {
        in_deadband_ = false;
        if (!lost_active_) { lost_active_ = true; lost_start_ts_ = meas_ts; reacq_count_ = 0; std::cout << "DBG lost_target\n"; }
        double bp = has_last_ ? last_pitch_ : gimbal_state_.pitch, by = has_last_ ? last_yaw_ : gimbal_state_.yaw;
        double pc = bp, yc = by;
        double lost_ms = static_cast<double>(meas_ts - lost_start_ts_);
        bool enter_scan = cfg_.scan_enable && (cfg_.scan_enter_delay_ms <= 0.0 || lost_ms >= cfg_.scan_enter_delay_ms);
        if (enter_scan) {
            if (!scanning_) {
                double offset = 0.0;
                if (has_last_meas_) {
                    double du = last_uv_.x - boresight_.u_L, dv = last_uv_.y - boresight_.v_L;
                    double u = cfg_.yaw_sign * du, v = cfg_.pitch_sign * dv;
                    if (std::abs(u) > 1e-6 || std::abs(v) > 1e-6) {
                        if (toLower(cfg_.scan_pattern) == "spiral") { u /= std::max(1e-3, cfg_.scan_k_yaw); v /= std::max(1e-3, cfg_.scan_k_pitch); }
                        offset = std::atan2(v, u);
                    }
                }
                scan_center_pitch_ = bp; scan_center_yaw_ = by; scan_phase_ = 0.0; scan_phase_offset_ = offset; scan_dir_ = 1; scanning_ = true;
                std::cout << "DBG scan_begin\n";
            }
            if (toLower(cfg_.scan_pattern) == "spiral") {
                double t = scan_phase_, a = std::max(1e-3, cfg_.scan_spacing_deg) / (2.0 * M_PI);
                double r = a * t, rm = std::max(cfg_.scan_spacing_deg, cfg_.scan_r_max_deg);
                double kx = std::max(1e-3, cfg_.scan_k_yaw), ky = std::max(1e-3, cfg_.scan_k_pitch), psi = t + scan_phase_offset_;
                double dx, dy;
                if (r < rm) { dx = kx*(a*std::cos(psi)-r*std::sin(psi)); dy = ky*(a*std::sin(psi)+r*std::cos(psi)); }
                else { r = rm; dx = -kx*r*std::sin(psi); dy = ky*r*std::cos(psi); }
                double ds = std::max(std::sqrt(dx*dx+dy*dy), kMinScanDs);
                t += (std::max(1e-3, cfg_.scan_speed_deg_s) / ds) * dt_s;
                scan_phase_ = t; r = a * t;
                pc = scan_center_pitch_ + ky * std::min(r, rm) * std::sin(t + scan_phase_offset_);
                yc = scan_center_yaw_ + kx * std::min(r, rm) * std::cos(t + scan_phase_offset_);
            } else {
                scan_phase_ += 2.0 * M_PI * cfg_.scan_rate_hz * dt_s;
                if (scan_phase_ > 2.0 * M_PI) scan_phase_ = std::fmod(scan_phase_, 2.0 * M_PI);
                pc = scan_center_pitch_ + cfg_.scan_radius_deg * std::sin(scan_phase_ + scan_phase_offset_);
                yc = scan_center_yaw_ + cfg_.scan_radius_deg * std::cos(scan_phase_ + scan_phase_offset_);
            }
        }
        double ms = cfg_.max_angle_rate * dt_s;
        pc = clamp(pc, bp - ms, bp + ms); yc = clamp(yc, by - ms, by + ms);
        cmd.yaw_vel = yc; cmd.pitch_vel = pc; cmd.is_absolute = true;
        last_pitch_ = pc; last_yaw_ = yc; has_last_ = true;
        DebugContext::getInstance().setTarget2D(Vec2::Zero(), false);
        return cmd;
    }
    if (lost_active_) {
        reacq_count_++;
        if (reacq_count_ >= std::max(1, cfg_.scan_reacq_confirm_frames)) { scanning_ = false; lost_active_ = false; reacq_count_ = 0; std::cout << "DBG reacquired\n"; }
    }
    double du = uv.x - boresight_.u_L, dv = uv.y - boresight_.v_L;
    double abs_du = std::abs(du), abs_dv = std::abs(dv);
    double db_enter = std::max(0.0, cfg_.deadband_px), db_exit = db_enter + kDeadbandHystPx;
    if (in_deadband_) { if (abs_du > db_exit || abs_dv > db_exit) in_deadband_ = false; }
    else { if (abs_du < db_enter && abs_dv < db_enter) in_deadband_ = true; }
    if (in_deadband_) {
        if (has_last_) { cmd.yaw_vel = last_yaw_; cmd.pitch_vel = last_pitch_; cmd.is_absolute = true; }
        last_uv_ = uv; last_meas_ts_ = meas_ts; has_last_meas_ = true;
        DebugContext::getInstance().setTarget2D(Vec2(du/320.*200., dv/240.*150.), true);
        return cmd;
    }
    double dyaw = cfg_.yaw_sign * std::atan(du / cam_model_.fx) * kRadToDeg;
    double dpitch = cfg_.pitch_sign * std::atan(dv / cam_model_.fy) * kRadToDeg;
    double yc = gimbal_state_.yaw + cfg_.kp * dyaw, pc = gimbal_state_.pitch + cfg_.kp * dpitch;
    if (cfg_.use_damping && cfg_.damping_kd > 0.0 && has_last_meas_) {
        double dms = static_cast<double>(meas_ts - last_meas_ts_);
        if (dms > 0.0 && dms <= cfg_.damping_dt_max_ms) {
            double dt = dms / 1000.0, yr = cfg_.yaw_sign * (uv.x-last_uv_.x)/dt/cam_model_.fx*kRadToDeg;
            double pr = cfg_.pitch_sign * (uv.y-last_uv_.y)/dt/cam_model_.fy*kRadToDeg;
            yc -= cfg_.damping_kd * yr; pc -= cfg_.damping_kd * pr;
        }
    }
    if (cfg_.use_velocity_ff && has_last_meas_) {
        double dms = static_cast<double>(meas_ts - last_meas_ts_);
        if (dms > 0.0 && dms <= cfg_.ff_dt_max_ms) {
            double dt = dms / 1000.0, yr = cfg_.yaw_sign * (uv.x-last_uv_.x)/dt/cam_model_.fx*kRadToDeg;
            double pr = cfg_.pitch_sign * (uv.y-last_uv_.y)/dt/cam_model_.fy*kRadToDeg;
            last_ff_yaw_rate_ = cfg_.ff_alpha*yr + (1.0-cfg_.ff_alpha)*last_ff_yaw_rate_;
            last_ff_pitch_rate_ = cfg_.ff_alpha*pr + (1.0-cfg_.ff_alpha)*last_ff_pitch_rate_;
            cmd.yaw_vel = clamp(last_ff_yaw_rate_, -cfg_.ff_rate_max, cfg_.ff_rate_max);
            cmd.pitch_vel = clamp(last_ff_pitch_rate_, -cfg_.ff_rate_max, cfg_.ff_rate_max);
        }
    }
    if (has_last_) {
        yc = cfg_.lowpass_alpha*yc + (1.0-cfg_.lowpass_alpha)*last_yaw_;
        pc = cfg_.lowpass_alpha*pc + (1.0-cfg_.lowpass_alpha)*last_pitch_;
        double ms = cfg_.max_angle_rate * dt_s;
        yc = clamp(yc, last_yaw_-ms, last_yaw_+ms); pc = clamp(pc, last_pitch_-ms, last_pitch_+ms);
    }
    last_pitch_ = pc; last_yaw_ = yc; has_last_ = true;
    last_uv_ = uv; last_meas_ts_ = meas_ts; has_last_meas_ = true;
    cmd.yaw_vel = yc; cmd.pitch_vel = pc; cmd.is_absolute = true;
    DebugContext::getInstance().setTarget2D(Vec2(du/320.*200., dv/240.*150.), true);
    DebugContext::getInstance().pushCurveData("err_x", du);
    DebugContext::getInstance().pushCurveData("err_y", dv);
    DebugContext::getInstance().pushCurveData("cmd_yaw", yc);
    DebugContext::getInstance().pushCurveData("cmd_pitch", pc);
    return cmd;
}

bool LaserAimerSolver::loadConfig(const std::string& path, ControlConfig* cfg, CameraModel* cam, Boresight* bs) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) { std::cerr << "Failed to open config: " << path << "\n"; return false; }
    if (cfg) {
        fs["kp"] >> cfg->kp; fs["deadband_px"] >> cfg->deadband_px; fs["max_angle_rate"] >> cfg->max_angle_rate;
        fs["lowpass_alpha"] >> cfg->lowpass_alpha;
        if (!fs["yaw_sign"].empty()) fs["yaw_sign"] >> cfg->yaw_sign;
        if (!fs["pitch_sign"].empty()) fs["pitch_sign"] >> cfg->pitch_sign;
        if (!fs["use_velocity_ff"].empty()) fs["use_velocity_ff"] >> cfg->use_velocity_ff;
        if (!fs["ff_alpha"].empty()) fs["ff_alpha"] >> cfg->ff_alpha;
        if (!fs["use_damping"].empty()) fs["use_damping"] >> cfg->use_damping;
        if (!fs["damping_source"].empty()) fs["damping_source"] >> cfg->damping_source;
        if (!fs["damping_kd"].empty()) fs["damping_kd"] >> cfg->damping_kd;
        if (!fs["scan_enable"].empty()) fs["scan_enable"] >> cfg->scan_enable;
        if (!fs["scan_radius_deg"].empty()) fs["scan_radius_deg"] >> cfg->scan_radius_deg;
        if (!fs["scan_rate_hz"].empty()) fs["scan_rate_hz"] >> cfg->scan_rate_hz;
        if (!fs["scan_pattern"].empty()) fs["scan_pattern"] >> cfg->scan_pattern;
        if (!fs["scan_spacing_deg"].empty()) fs["scan_spacing_deg"] >> cfg->scan_spacing_deg;
        if (!fs["scan_speed_deg_s"].empty()) fs["scan_speed_deg_s"] >> cfg->scan_speed_deg_s;
        if (!fs["scan_r_max_deg"].empty()) fs["scan_r_max_deg"] >> cfg->scan_r_max_deg;
        if (!fs["scan_k_yaw"].empty()) fs["scan_k_yaw"] >> cfg->scan_k_yaw;
        if (!fs["scan_k_pitch"].empty()) fs["scan_k_pitch"] >> cfg->scan_k_pitch;
        if (!fs["scan_enter_delay_ms"].empty()) fs["scan_enter_delay_ms"] >> cfg->scan_enter_delay_ms;
        if (!fs["startup_check_frames"].empty()) fs["startup_check_frames"] >> cfg->startup_check_frames;
        if (!fs["startup_prep_ms"].empty()) fs["startup_prep_ms"] >> cfg->startup_prep_ms;
    }
    if (cam) { fs["fx"] >> cam->fx; fs["fy"] >> cam->fy; fs["cx"] >> cam->cx; fs["cy"] >> cam->cy; }
    if (bs) { fs["u_L"] >> bs->u_L; fs["v_L"] >> bs->v_L; }
    return true;
}
} // namespace aim
