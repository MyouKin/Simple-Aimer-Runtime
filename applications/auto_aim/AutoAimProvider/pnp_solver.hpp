#ifndef AUTO_AIM__PNP_SOLVER_HPP
#define AUTO_AIM__PNP_SOLVER_HPP
/// @file pnp_solver.hpp
/// @brief PnP 3D 姿态解算器 — 移植自 spr_vision_try/tasks/auto_aim/solver.cpp
///
/// 坐标变换链：Camera → Gimbal → World
///   solvePnP → camera 系位姿 (rvec, tvec)
///   → R_camera2gimbal, t_camera2gimbal → gimbal 系
///   → R_gimbal2world → world 系

#include "../types.hpp"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace auto_aim {

class PnPSolver {
public:
  /// @param config_path  配置文件路径（读取 camera_matrix / R_camera2gimbal 等）
  explicit PnPSolver(const std::string & config_path);

  /// @brief 对单个 Armor 执行 PnP 解算，填入 xyz_in_world / ypr_in_world / ypd_in_world
  void solve(Armor & armor) const;

  /// @brief 重投影：给定世界坐标的 xyz + yaw → 反算到像素坐标（4 个角点）
  ///        用于可视化调试
  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw,
    ArmorType type, ArmorName name) const;

  /// @brief 通过 IMU 四元数更新世界变换矩阵（云台 → 世界）
  /// @param q  IMU 反馈的四元数（IMU 本体 → 绝对坐标系）
  void setRgimbal2world(const Eigen::Quaterniond & q);

  Eigen::Matrix3d Rgimbal2world() const { return R_gimbal2world_; }

private:
  cv::Mat camera_matrix_;       // 相机内参 3x3
  cv::Mat distort_coeffs_;      // 畸变系数 1x5

  Eigen::Matrix3d R_gimbal2imubody_;   // 云台 → IMU 本体
  Eigen::Matrix3d R_camera2gimbal_;    // 相机 → 云台
  Eigen::Vector3d t_camera2gimbal_;    // 相机 → 云台 平移

  Eigen::Matrix3d R_gimbal2world_ = Eigen::Matrix3d::Identity();  // 云台 → 世界（由 IMU 动态更新）

  /// @brief 在 ±70° 范围内搜索最优 yaw，使重投影误差最小
  void optimize_yaw(Armor & armor) const;

  /// @brief 计算给定 yaw 下的重投影误差
  double armor_reprojection_error(const Armor & armor, double yaw, double inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PNP_SOLVER_HPP
