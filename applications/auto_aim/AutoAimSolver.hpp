#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP
/// @file AutoAimSolver.hpp
/// @brief 控制指令解算器 — 弹道瞄准 + TinyMPC 云台控制 + 开火判断
///
/// 移植自 spr_vision_try：
///   - tasks/auto_aim/aimer.cpp            — 弹道迭代 + 瞄准点选择
///   - tasks/auto_aim/planner/planner.cpp  — MPC (TinyMPC) yaw/pitch 求解 + 开火
///
/// ## 控制流程
///
///   target (Selector) + system_state (System)
///     → 延迟补偿 + 前向预测
///     → 选择瞄准点 (choose_aim_xyza)
///     → 弹道解算 (aim_ballistic)
///     → 飞行时间前向预测
///     → 生成参考轨迹 (get_trajectory)
///     → yaw MPC 求解
///     → pitch MPC 求解
///     → 开火判断 (fire_thresh_)
///     → 输出 GimbalCommand

#include "../../include/pipeline/Solver.hpp"
#include "../../include/tools/trajectory.hpp"
#include "../../include/tools/tinympc_solver.hpp"
#include "types.hpp"

#include <Eigen/Dense>
#include <memory>
#include <string>

namespace auto_aim {

// MPC 参数（对应 spr_vision_try 的 planner.hpp）
constexpr double MPC_DT          = 0.01;   // 离散时间步长 (s)
constexpr int    MPC_HALF_HORIZON = 50;    // 半时域
constexpr int    MPC_HORIZON      = MPC_HALF_HORIZON * 2;  // 全时域 (100 步 = 1.0s)
constexpr int    MPC_SHOOT_OFFSET = 2;     // 开火判定的前向步数偏移

using TrajectoryMatrix = Eigen::Matrix<double, 4, MPC_HORIZON>;  // [yaw, yaw_vel, pitch, pitch_vel]

class AutoAimSolver : public aim::Solver<AutoAimSystemState> {
public:
  explicit AutoAimSolver(const std::string & config_path);

  aim::Command solve(
    const aim::FinalTargetState & target,
    const AutoAimSystemState & system_state) override;

  const Eigen::Vector4d & debugAimPoint() const { return debug_aim_xyza_; }

private:
  // ---- 配置 ----
  double yaw_offset_    = 0.0;
  double pitch_offset_  = 0.0;
  double fire_thresh_   = 2.0;         // 开火角度阈值 (rad)
  double decision_speed_     = 3.0;    // 高低速判定角速度 (rad/s)
  double high_speed_delay_time_ = 0.1; // 高速延迟 (s)
  double low_speed_delay_time_  = 0.05;

  // ---- MPC 求解器 ----
  std::unique_ptr<aim::tools::TinyMPCSolver> yaw_solver_;
  std::unique_ptr<aim::tools::TinyMPCSolver> pitch_solver_;

  // ---- 调试 ----
  Eigen::Vector4d debug_aim_xyza_{0, 0, 0, 0};

  // ---- 方法 ----
  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  /// @brief 从 EKF 状态提取装甲板列表 [x, y, z, yaw]
  std::vector<Eigen::Vector4d> armor_xyza_list(const AutoAimSystemState & state) const;

  /// @brief 选择瞄准点
  Eigen::Vector4d choose_aim_xyza(const AutoAimSystemState & state) const;

  /// @brief 弹道解算 → [yaw, pitch]（含 offset）
  Eigen::Vector2d aim_ballistic(const Eigen::Vector4d & aim_xyza, double bullet_speed) const;

  /// @brief 对 EKF 状态副本前向预测
  void predict_forward(AutoAimSystemState & state_copy, double dt) const;

  /// @brief 生成参考轨迹（原 Planner::get_trajectory）
  TrajectoryMatrix get_trajectory(AutoAimSystemState & state_copy,
                                  double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP