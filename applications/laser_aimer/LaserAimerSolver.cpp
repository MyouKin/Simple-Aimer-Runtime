#include "LaserAimerSolver.hpp"
#include "../../include/core/Registry.hpp"
#include "../../include/core/DebugContext.hpp"
#include <cmath>

namespace aim {

LaserAimerSolver::LaserAimerSolver() {
    auto& reg = ParameterRegistry::getInstance();
    
    // According to BreCaspian architecture: Parallax compensation by shifting the aiming center
    reg.registerFloat("Boresight U (px)", &boresight_u_, 0.0, 1280.0);
    reg.registerFloat("Boresight V (px)", &boresight_v_, 0.0, 720.0);
    
    reg.registerFloat("Camera fx", &fx_, 200.0, 3000.0);
    reg.registerFloat("Camera fy", &fy_, 200.0, 3000.0);

    reg.registerFloat("Control Kp", &kp_, 0.0, 5.0);
    reg.registerFloat("Damping Kd", &damping_kd_, 0.0, 1.0);
    reg.registerFloat("Deadband (px)", &deadband_px_, 0.0, 50.0);
}

GimbalCommand LaserAimerSolver::solve(const TargetState& target) {
    GimbalCommand cmd;
    cmd.is_absolute = false; // We output angular velocity/delta

    if (!target.valid || !target.has_image_point) {
        cmd.yaw_vel = 0.0;
        cmd.pitch_vel = 0.0;
        has_last_ = false;
        DebugContext::getInstance().setTarget2D(Vec2::Zero(), false);
        return cmd;
    }

    // BreCaspian Controller Logic: Error relative to calibrated Boresight (Parallax)
    double du = target.image_point.x() - boresight_u_;
    double dv = target.image_point.y() - boresight_v_;

    // Map pixel error to angular error (rad) -> (deg)
    // Note: arctan(pixel_err / focal_length) is exactly what's used in BreCaspian
    double dyaw_rad = std::atan(du / fx_);
    double dpitch_rad = std::atan(dv / fy_);

    // Implement Deadband (Hysteresis omitted for simplicity, pure threshold here)
    if (std::abs(du) < deadband_px_ && std::abs(dv) < deadband_px_) {
        if (has_last_) {
            cmd.yaw_vel = last_yaw_cmd_;
            cmd.pitch_vel = last_pitch_cmd_;
        } else {
            cmd.yaw_vel = 0.0;
            cmd.pitch_vel = 0.0;
        }
        return cmd;
    }

    // Simple P Controller logic equivalent to what they calculate as "Position control" delta
    double yaw_cmd = kp_ * dyaw_rad;
    double pitch_cmd = kp_ * dpitch_rad;

    // Output is command delta/velocity
    cmd.yaw_vel = yaw_cmd;
    cmd.pitch_vel = pitch_cmd;

    last_yaw_cmd_ = yaw_cmd;
    last_pitch_cmd_ = pitch_cmd;
    has_last_ = true;

    // Push Curve Data
    DebugContext::getInstance().pushCurveData("Boresight Err X", du);
    DebugContext::getInstance().pushCurveData("Boresight Err Y", dv);
    DebugContext::getInstance().pushCurveData("Cmd Yaw", cmd.yaw_vel);
    DebugContext::getInstance().pushCurveData("Cmd Pitch", cmd.pitch_vel);

    // Visualize relative to the Boresight (0,0 maps to Boresight)
    double norm_x = du / 320.0 * 200.0;
    double norm_y = dv / 240.0 * 150.0;
    DebugContext::getInstance().setTarget2D(Vec2(norm_x, norm_y), true);

    return cmd;
}

} // namespace aim