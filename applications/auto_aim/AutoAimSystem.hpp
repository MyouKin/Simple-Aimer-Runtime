#ifndef AUTO_AIM__SYSTEM_HPP
#define AUTO_AIM__SYSTEM_HPP
/// @file AutoAimSystem.hpp
/// @brief 系统状态维护层 — EKF 预测/更新 + 追踪器状态机
///
/// 移植自 spr_vision_try：
///   - tasks/auto_aim/target.cpp   — Target 类（EKF 预测/更新/发散检测）
///   - tasks/auto_aim/tracker.cpp  — Tracker 类（状态机 / set_target / update_target）
///
/// 作为 Simple-Aimer-Runtime 的 System<ArmorList, AutoAimSystemState> 实现。

#include "../../include/pipeline/System.hpp"
#include "../../include/tools/extended_kalman_filter.hpp"
#include "types.hpp"

#include <opencv2/opencv.hpp>
#include <string>

namespace auto_aim {

class AutoAimSystem : public aim::System<ArmorList, AutoAimSystemState> {
public:
  /// @param config_path  YAML 配置文件路径（读取 enemy_color / min_detect_count 等）
  explicit AutoAimSystem(const std::string & config_path);

  // ---- System 接口 ----
  void update(const ArmorList & armors) override;
  void updateSelfState(const aim::SelfState & self) override;
  const AutoAimSystemState & getState() const override { return state_; }

  /// @brief 当前追踪器状态字符串（调试用，对应原 Tracker::state()）
  std::string trackStateStr() const;

private:
  // ========================================================================
  // 状态维护
  // ========================================================================
  AutoAimSystemState state_;

  // 内部 EKF 对象（原 Target::ekf_，这里作为内部引擎；state_.ekf_x/P 同步副本）
  aim::tools::ExtendedKalmanFilter ekf_;

  // 追踪器参数（原 Tracker 构造时从 YAML 读取）
  Color enemy_color_;
  int min_detect_count_;
  int max_temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;

  // ---- EKF 调试可视化 ---- 
  cv::Mat debug_camera_matrix_;
  cv::Mat debug_distort_coeffs_;
  Eigen::Matrix3d debug_R_camera2gimbal_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d debug_t_camera2gimbal_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d debug_R_gimbal2world_  = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d debug_R_gimbal2imubody_ = Eigen::Matrix3d::Identity();
  cv::Mat last_raw_frame_;

  // ========================================================================
  // 移植自 Target 的方法（EKF 预测/更新）
  // ========================================================================

  /// @brief EKF 状态传播（原 Target::predict(dt)）
  void ekf_predict(double dt);

  /// @brief EKF 观测更新（原 Target::update_ypda(armor, id)）
  void ekf_update_ypda(const Armor & armor, int id);

  /// @brief 装甲板高度匹配（原 Target::match_armor_id）
  int match_armor_id(double z_obs) const;

  /// @brief 计算第 id 块装甲板的 3D 坐标（原 Target::h_armor_xyz）
  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;

  /// @brief 观测雅可比（原 Target::h_jacobian）
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;

  /// @brief 发散检测（原 Target::diverged）
  bool ekf_diverged() const;

  /// @brief 收敛检测（原 Target::convergened）
  bool ekf_converged();

  // ========================================================================
  // 移植自 Tracker 的方法（状态机 / 目标管理）
  // ========================================================================

  /// @brief 状态机（原 Tracker::state_machine）
  void state_machine(bool found);

  /// @brief 初始化新目标（原 Tracker::set_target）
  bool set_target(ArmorList & armors, std::chrono::steady_clock::time_point t);

  /// @brief 更新已有目标（原 Tracker::update_target）
  bool update_target(ArmorList & armors, std::chrono::steady_clock::time_point t);

  // ========================================================================
  // 辅助
  // ========================================================================

  /// @brief 同步 EKF 内部状态 → state_（每次 predict/update 后调用）
  void sync_state_from_ekf();

  /// @brief 将 EKF 预测的全部 N 块装甲板重投影到图像上，推送到 DebugContext("EKF")
  void push_ekf_debug_image();

  /// @brief 从 EKF 状态获取全部装甲板的 [x, y, z, yaw] 列表（调试用）
  std::vector<Eigen::Vector4d> ekf_armor_xyza_list() const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SYSTEM_HPP