#ifndef AIM_APP_LASER_AIMER_SOLVER_HPP
#define AIM_APP_LASER_AIMER_SOLVER_HPP

#include "../../include/pipeline/Solver.hpp"

namespace aim {

class LaserAimerSolver : public Solver {
public:
    LaserAimerSolver();
    GimbalCommand solve(const TargetState& target) override;

private:
    // PID Parameters
    double kp_ = 0.5;
    double damping_kd_ = 0.1;
    
    // Boresight Parallax Offset (relative to camera center)
    // The concept used by BreCaspian: the target center isn't the physical image center (u_L, v_L).
    double boresight_u_ = 320.0;
    double boresight_v_ = 240.0;

    // Camera Intrinsic
    double fx_ = 800.0;
    double fy_ = 800.0;

    // Optional Deadband to prevent jitter when target is roughly centered
    double deadband_px_ = 5.0;

    // Internal state
    double last_pitch_cmd_ = 0.0;
    double last_yaw_cmd_ = 0.0;
    bool has_last_ = false;
};

} // namespace aim

#endif // AIM_APP_LASER_AIMER_SOLVER_HPP