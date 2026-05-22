#include "LaserAimerSolver.hpp"

#include "../../include/core/DebugContext.hpp"

#include <cmath>

namespace laser_aimer {

LaserAimerSolver::LaserAimerSolver(const LaserAimerConfig & cfg)
  : cfg_(cfg), controller_(cfg.control) {}

aim::Command LaserAimerSolver::solve(const aim::FinalTargetState & target,
                                     const LaserAimerState & system_state) {
  TargetMeasurement meas;
  meas.timestamp_ms = system_state.last_input.timestamp_ms > 0 ?
    system_state.last_input.timestamp_ms : nowMs();
  if (target.valid && target.has_image_point) {
    meas.valid = true;
    meas.uv = {
      static_cast<float>(target.image_point.x()),
      static_cast<float>(target.image_point.y())};
    meas.confidence = system_state.filtered_measurement.confidence;
  }

  GimbalState gimbal = toGimbalState(system_state.self);
  GimbalCommand out = controller_.update(meas, cfg_.camera_model, cfg_.boresight, gimbal);

  aim::DebugContext::getInstance().pushCurveData("cmd_yaw_deg", out.yaw);
  aim::DebugContext::getInstance().pushCurveData("cmd_pitch_deg", out.pitch);
  aim::DebugContext::getInstance().pushCurveData("meas_valid", meas.valid ? 1.0F : 0.0F);

  aim::Command cmd = toAimCommand(out);
  if (meas.valid) {
    const double du = meas.uv.x - cfg_.boresight.u_l;
    const double dv = meas.uv.y - cfg_.boresight.v_l;
    cmd.is_fine_aiming =
      std::abs(du) <= cfg_.control.deadband_px &&
      std::abs(dv) <= cfg_.control.deadband_px;
  }
  return cmd;
}

}  // namespace laser_aimer
