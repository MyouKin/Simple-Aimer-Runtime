/// @file trajectory.hpp
/// @brief 理想抛物线弹道模型 — 忽略空气阻力，解析求解飞行时间和发射俯仰角
///
/// ## 模型
///
/// 从弹道方程直接推导：
///   d = v₀·cos(θ)·t          (水平匀速)
///   h = v₀·sin(θ)·t − ½g·t²  (竖直匀减速)
///
/// 消去 t 得到关于 tan(θ) 的二次方程，取高抛解（稳定性更好）。
///
/// ## 适用范围
///
///   - 短距离（< 50m）：空气阻力可忽略，精度足够
///   - 长距离：需考虑空气阻力，用数值积分替代
///   - 弹丸初速需已知（通常由云台下位机测量并反馈，v₀ = 15~30 m/s）
///
/// ## 用法
///
/// @code
/// #include "tools/trajectory.hpp"
///
/// double bullet_speed = 22.0;     // m/s
/// double distance = 5.0;          // 水平距离 (m)
/// double height   = 0.3;         // 目标高度 (m)
///
/// aim::tools::Trajectory traj(bullet_speed, distance, height);
/// if (traj.unsolvable) {
///   // 目标超出射程
/// } else {
///   double fly_time = traj.fly_time;  // 飞行时间 (s)，用于前向预测
///   double pitch    = traj.pitch;     // 发射俯仰角 (rad, 抬头为正)
/// }
/// @endcode
///
/// ## 在多任务中的复用
///
/// 本模型是纯数学工具，不依赖任何应用层类型。所有需要弹道解算的瞄准任务
/// （auto_aim、laser_aimer 等）均可直接使用。

#ifndef AIM_TOOLS_TRAJECTORY_HPP
#define AIM_TOOLS_TRAJECTORY_HPP

#include <cmath>

namespace aim {
namespace tools {

/// @brief 不考虑空气阻力的弹道解算
struct Trajectory {
  bool unsolvable = false;
  double fly_time = 0.0;  // 飞行时间 (秒)
  double pitch    = 0.0;  // 发射俯仰角 (rad, 抬头为正)

  /// @param v0 子弹初速度 (m/s)
  /// @param d  目标水平距离 (m)
  /// @param h  目标竖直高度 (m)
  Trajectory(double v0, double d, double h) {
    constexpr double g = 9.81;
    double v0_sq = v0 * v0;
    double d_sq  = d * d;

    double discriminant = v0_sq * v0_sq - g * (g * d_sq + 2.0 * h * v0_sq);
    if (discriminant < 0) {
      unsolvable = true;
      return;
    }
    double sqrt_D = std::sqrt(discriminant);

    // 高抛解与低平解两种，取飞行时间短的那个（更直接的弹道）
    double tan_high = (v0_sq + sqrt_D) / (g * d);
    double tan_low  = (v0_sq - sqrt_D) / (g * d);

    double theta_high = std::atan(tan_high);
    double theta_low  = std::atan(tan_low);

    double ft_high = d / (v0 * std::cos(theta_high));
    double ft_low  = d / (v0 * std::cos(theta_low));

    if (ft_low > 0 && ft_low <= ft_high) {
      pitch = theta_low;  fly_time = ft_low;
    } else if (ft_high > 0) {
      pitch = theta_high; fly_time = ft_high;
    } else {
      unsolvable = true;
    }
  }
};

}  // namespace tools
}  // namespace aim

#endif  // AIM_TOOLS_TRAJECTORY_HPP