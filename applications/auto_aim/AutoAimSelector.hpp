#ifndef AUTO_AIM__SELECTOR_HPP
#define AUTO_AIM__SELECTOR_HPP
/// @file AutoAimSelector.hpp
/// @brief 目标选择器 — 从 AutoAimSystemState 中提取统一的 TargetState
///
/// 对应 spr_vision_try 中 Tracker::track() 返回 target_ 后、传入 Aimer 前的步骤。
/// 将 EKF 状态转换为 Simple-Aimer-Runtime 的标准 FinalTargetState。

#include "../../include/pipeline/Selector.hpp"
#include "types.hpp"

namespace auto_aim {

class AutoAimSelector : public aim::Selector<AutoAimSystemState> {
public:
  aim::FinalTargetState select(const AutoAimSystemState & state) override {
    aim::FinalTargetState target;
    target.timestamp = std::chrono::high_resolution_clock::now();

    if (!state.has_valid_target()) {
      target.valid = false;
      return target;
    }

    // 从 EKF 状态提取 3D 位置和速度
    // EKF 状态: [x, vx, y, vy, z, vz, angle, ω, r, l, h]
    if (state.ekf_x.size() >= 7) {
      target.has_position = true;
      target.position = Eigen::Vector3d(state.ekf_x[0], state.ekf_x[2], state.ekf_x[4]);
    }
    if (state.ekf_x.size() >= 6) {
      target.has_velocity = true;
      target.velocity = Eigen::Vector3d(state.ekf_x[1], state.ekf_x[3], state.ekf_x[5]);
    }
    // EKF 不直接维护加速度，使用角速度作为补充信息
    if (state.ekf_x.size() >= 8) {
      target.has_euler = true;
      target.euler = Eigen::Vector3d(0, 0, state.ekf_x[6]);  // yaw 存入 euler[2]
    }

    target.valid = true;
    target.frame_id = ARMOR_NAMES[static_cast<size_t>(state.target_name)];
    return target;
  }
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SELECTOR_HPP