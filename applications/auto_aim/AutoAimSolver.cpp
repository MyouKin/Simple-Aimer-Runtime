/// @file AutoAimSolver.cpp
/// @brief Solver 实现 — TinyMPC 云台控制 + 弹道瞄准 + 开火判断
///
/// 移植自 spr_vision_try 的：
///   - aimer.cpp     — 弹道迭代、瞄准点选择
///   - planner.cpp   — MPC 求解、参考轨迹生成、开火判定
///
/// ## 开火逻辑
///
/// 开火判断 (plan.fire) 在 solve() 的最后一步完成：
///   1. 计算参考轨迹 (get_trajectory) — 目标在未来 HORIZON 步的理想 yaw/pitch 序列
///   2. MPC 求解得到优化后的 yaw/pitch 轨迹
///   3. 在 HALF_HORIZON + SHOOT_OFFSET 处比较参考轨迹与 MPC 轨迹的 2D 欧氏距离
///   4. 若距离 < fire_thresh_，则 cmd.fire = true
///
/// 直觉：当云台能紧密跟踪目标预测轨迹时（误差小于阈值），可以开火。

#include "AutoAimSolver.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace {

constexpr double kPi    = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

inline double limit_rad(double a) {
  while (a > kPi)  a -= kTwoPi;
  while (a < -kPi) a += kTwoPi;
  return a;
}

}  // anonymous namespace

namespace auto_aim {

// ============================================================================
// 构造 — 初始化 MPC 求解器
// ============================================================================

AutoAimSolver::AutoAimSolver(const std::string & config_path) {
  cv::FileStorage fs(config_path, cv::FileStorage::READ);

  if (fs.isOpened()) {
    double yoff = 0, poff = 0;
    fs["yaw_offset"]   >> yoff;  yaw_offset_   = yoff / 57.3;
    fs["pitch_offset"] >> poff;  pitch_offset_ = poff / 57.3;
    fs["fire_thresh"]           >> fire_thresh_;
    fs["decision_speed"]        >> decision_speed_;
    fs["high_speed_delay_time"] >> high_speed_delay_time_;
    fs["low_speed_delay_time"]  >> low_speed_delay_time_;
    fs.release();
  }

  setup_yaw_solver(config_path);
  setup_pitch_solver(config_path);
}

void AutoAimSolver::setup_yaw_solver(const std::string & config_path) {
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  double max_yaw_acc = 50.0;   // rad/s²
  Eigen::Vector2d Q_yaw(1.0, 0.1);
  Eigen::VectorXd R_yaw(1); R_yaw << 0.01;
  if (fs.isOpened()) {
    fs["max_yaw_acc"] >> max_yaw_acc;
    cv::FileNode qn = fs["Q_yaw"];
    if (!qn.empty()) { Q_yaw(0) = qn[0]; Q_yaw(1) = qn[1]; }
    cv::FileNode rn = fs["R_yaw"];
    if (!rn.empty()) R_yaw(0) = rn[0];
    fs.release();
  }

  yaw_solver_ = std::make_unique<aim::tools::TinyMPCSolver>(MPC_DT, MPC_HORIZON);
  yaw_solver_->setup(max_yaw_acc, Q_yaw, R_yaw);
}

void AutoAimSolver::setup_pitch_solver(const std::string & config_path) {
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  double max_pitch_acc = 30.0;
  Eigen::Vector2d Q_pitch(1.0, 0.1);
  Eigen::VectorXd R_pitch(1); R_pitch << 0.01;
  if (fs.isOpened()) {
    fs["max_pitch_acc"] >> max_pitch_acc;
    cv::FileNode qn = fs["Q_pitch"];
    if (!qn.empty()) { Q_pitch(0) = qn[0]; Q_pitch(1) = qn[1]; }
    cv::FileNode rn = fs["R_pitch"];
    if (!rn.empty()) R_pitch(0) = rn[0];
    fs.release();
  }

  pitch_solver_ = std::make_unique<aim::tools::TinyMPCSolver>(MPC_DT, MPC_HORIZON);
  pitch_solver_->setup(max_pitch_acc, Q_pitch, R_pitch);
}

// ============================================================================
// solve() — 主入口，完全对应 spr Planner::plan()
// ============================================================================

aim::Command AutoAimSolver::solve(
  const aim::FinalTargetState & target,
  const AutoAimSystemState & system_state) {

  aim::Command cmd;

  if (!target.valid || !system_state.has_valid_target()) {
    return cmd;
  }

  // ---- 0. 弹丸速度 ----
  double bullet_speed = 22.0;  // m/s，实际应由 Actuator 反馈提供
  if (system_state.self.valid) {
    // TODO: 从 self 反馈中读取 bullet_speed（当前 SelfState 不含此字段）
  }
  if (bullet_speed < 10 || bullet_speed > 30) bullet_speed = 22.0;

  // ---- 1. 延迟补偿 ----
  double delay_time = (std::abs(system_state.ekf_x[7]) > decision_speed_)
                      ? high_speed_delay_time_ : low_speed_delay_time_;

  AutoAimSystemState state_copy = system_state;
  predict_forward(state_copy, delay_time);

  // ---- 2. 瞄准点 + 弹道 ----
  Eigen::Vector4d aim_xyza = choose_aim_xyza(state_copy);
  debug_aim_xyza_ = aim_xyza;

  Eigen::Vector3d xyz = aim_xyza.head<3>();
  double min_dist = xyz.head<2>().norm();

  aim::tools::Trajectory bullet_traj(bullet_speed, min_dist, xyz.z());
  if (bullet_traj.unsolvable) return cmd;

  // ---- 3. 飞行时间前向预测 ----
  predict_forward(state_copy, bullet_traj.fly_time);

  // ---- 4. 生成参考轨迹 ----
  double yaw0;
  TrajectoryMatrix traj;
  try {
    aim_xyza = choose_aim_xyza(state_copy);
    debug_aim_xyza_ = aim_xyza;
    yaw0 = aim_ballistic(aim_xyza, bullet_speed)(0);
    traj = get_trajectory(state_copy, yaw0, bullet_speed);
  } catch (const std::exception &) {
    return cmd;  // 目标不可解
  }

  // ---- 5. yaw MPC ----
  Eigen::Vector2d x0_yaw(traj(0, 0), traj(1, 0));
  yaw_solver_->solve(x0_yaw, traj.block(0, 0, 2, MPC_HORIZON));

  // ---- 6. pitch MPC ----
  Eigen::Vector2d x0_pitch(traj(2, 0), traj(3, 0));
  pitch_solver_->solve(x0_pitch, traj.block(2, 0, 2, MPC_HORIZON));

  // ---- 7. 输出指令 (HALF_HORIZON 处) ----
  cmd.yaw       = limit_rad(yaw_solver_->state(0, MPC_HALF_HORIZON) + yaw0);
  cmd.yaw_vel   = yaw_solver_->state(1, MPC_HALF_HORIZON);
  cmd.yaw_acc   = yaw_solver_->control(MPC_HALF_HORIZON);
  cmd.pitch     = pitch_solver_->state(0, MPC_HALF_HORIZON);
  cmd.pitch_vel = pitch_solver_->state(1, MPC_HALF_HORIZON);
  cmd.pitch_acc = pitch_solver_->control(MPC_HALF_HORIZON);

  // ---- 8. 开火判断 ----
  // 比较参考轨迹与 MPC 轨迹在 HALF_HORIZON + SHOOT_OFFSET 处的偏差
  int fire_step = MPC_HALF_HORIZON + MPC_SHOOT_OFFSET;
  if (fire_step < MPC_HORIZON) {
    double yaw_err   = traj(0, fire_step) - yaw_solver_->state(0, fire_step);
    double pitch_err = traj(2, fire_step) - pitch_solver_->state(0, fire_step);
    cmd.is_fine_aiming = (std::hypot(yaw_err, pitch_err) < fire_thresh_);
  }

  return cmd;
}

// ============================================================================
// get_trajectory — 生成参考轨迹（原 Planner::get_trajectory）
// ============================================================================

TrajectoryMatrix AutoAimSolver::get_trajectory(
  AutoAimSystemState & state_copy, double yaw0, double bullet_speed) {

  TrajectoryMatrix traj;

  // 回退到 -HALF_HORIZON 时刻
  predict_forward(state_copy, -MPC_DT * (MPC_HALF_HORIZON + 1));
  auto yaw_pitch_last = aim_ballistic(choose_aim_xyza(state_copy), bullet_speed);

  // 前进到 0 时刻
  predict_forward(state_copy, MPC_DT);
  auto yaw_pitch = aim_ballistic(choose_aim_xyza(state_copy), bullet_speed);

  // 生成 HORIZON 步轨迹
  for (int i = 0; i < MPC_HORIZON; ++i) {
    predict_forward(state_copy, MPC_DT);
    auto yaw_pitch_next = aim_ballistic(choose_aim_xyza(state_copy), bullet_speed);

    double yaw_vel   = limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * MPC_DT);
    double pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * MPC_DT);

    traj.col(i) << limit_rad(yaw_pitch(0) - yaw0), yaw_vel,
                   yaw_pitch(1), pitch_vel;

    yaw_pitch_last = yaw_pitch;
    yaw_pitch      = yaw_pitch_next;
  }

  return traj;
}

// ============================================================================
// 辅助方法（保留原有实现）
// ============================================================================

std::vector<Eigen::Vector4d> AutoAimSolver::armor_xyza_list(
  const AutoAimSystemState & state) const {

  std::vector<Eigen::Vector4d> result;
  const auto & x = state.ekf_x;

  for (int i = 0; i < state.armor_num; ++i) {
    auto angle = limit_rad(x[6] + i * 2 * kPi / state.armor_num);
    double armor_x, armor_y, armor_z;
    bool use_l_h = (state.armor_num == 4) && (i == 1 || i == 3);
    if (state.target_name == ArmorName::outpost && state.armor_num == 3 &&
        static_cast<int>(x.size()) >= 13) {
      auto r = x[8];
      armor_x = x[0] - r * std::cos(angle);
      armor_y = x[2] - r * std::sin(angle);
      armor_z = (i == 0) ? x[4] : (i == 1 ? x[11] : x[12]);
    } else {
      auto r = use_l_h ? x[8] + x[9] : x[8];
      armor_x = x[0] - r * std::cos(angle);
      armor_y = x[2] - r * std::sin(angle);
      armor_z = use_l_h ? x[4] + x[10] : x[4];
    }
    result.push_back({armor_x, armor_y, armor_z, angle});
  }
  return result;
}

Eigen::Vector4d AutoAimSolver::choose_aim_xyza(
  const AutoAimSystemState & state) const {

  auto xyza_list = armor_xyza_list(state);
  if (xyza_list.empty()) return {0, 0, 0, 0};

  if (state.target_name == ArmorName::outpost && state.armor_num == 3) {
    const Eigen::VectorXd & x = state.ekf_x;
    auto center_yaw = std::atan2(x[2], x[0]);
    int best_id = 0;
    double min_abs = 1e10;
    for (int i = 0; i < state.armor_num; ++i) {
      auto delta = limit_rad(xyza_list[i][3] - center_yaw);
      if (std::abs(delta) < min_abs) { min_abs = std::abs(delta); best_id = i; }
    }
    return xyza_list[best_id];
  }

  double min_dist = 1e10;
  Eigen::Vector4d best = xyza_list[0];
  for (const auto & xyza : xyza_list) {
    double dist = xyza.head<2>().norm();
    if (dist < min_dist) { min_dist = dist; best = xyza; }
  }
  return best;
}

Eigen::Vector2d AutoAimSolver::aim_ballistic(
  const Eigen::Vector4d & aim_xyza, double bullet_speed) const {

  Eigen::Vector3d xyz = aim_xyza.head<3>();
  double min_dist = xyz.head<2>().norm();
  double azim = std::atan2(xyz.y(), xyz.x());

  aim::tools::Trajectory traj(bullet_speed, min_dist, xyz.z());
  return {limit_rad(azim + yaw_offset_), -traj.pitch - pitch_offset_};
}

void AutoAimSolver::predict_forward(
  AutoAimSystemState & state_copy, double dt) const {

  if (dt == 0.0) return;

  int dim = static_cast<int>(state_copy.ekf_x.size());
  Eigen::MatrixXd F = Eigen::MatrixXd::Identity(dim, dim);
  F(0, 1) = dt; F(2, 3) = dt; F(4, 5) = dt; F(6, 7) = dt;
  state_copy.ekf_x = F * state_copy.ekf_x;
  state_copy.ekf_x[6] = limit_rad(state_copy.ekf_x[6]);
}

}  // namespace auto_aim