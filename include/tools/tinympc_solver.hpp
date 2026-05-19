/// @file tinympc_solver.hpp
/// @brief TinyMPC 求解器封装 — 运动学 MPC（双积分器模型）
///
/// ## 模型
///
/// 对 yaw 和 pitch 各使用独立的双积分器模型：
///   A = [[1, DT], [0, 1]]    B = [[0], [DT]]
///   状态: [角度, 角速度]      控制: 角加速度
///
/// ## 依赖
///
/// 需要 TinyMPC 库（https://github.com/TinyMPC/TinyMPC）。
/// 下载后放在任意位置，编译时定义 TINYMPC_INCLUDE_DIR：
///   cmake .. -DTINYMPC_INCLUDE_DIR=/path/to/TinyMPC/include
///
/// 若未定义 TINYMPC_INCLUDE_DIR，则编译为存根模式（solve() 返回零轨迹）。
///
/// ## 用法
///
/// @code
/// #include "tools/tinympc_solver.hpp"
///
/// double DT = 0.01;
/// int HORIZON = 50;
///
/// aim::tools::TinyMPCSolver yaw_solver(DT, HORIZON);
/// yaw_solver.setup(max_acc, Q, R);  // Q(2x1), R(1x1) 权重
///
/// // 每周期：
/// yaw_solver.solve(x0, x_ref);  // x0=[yaw, yaw_vel], x_ref=参考轨迹
/// double yaw_cmd   = yaw_solver.state(0, HALF_HORIZON);
/// double yaw_vel   = yaw_solver.state(1, HALF_HORIZON);
/// double yaw_acc   = yaw_solver.control(0, HALF_HORIZON);
/// @endcode

#ifndef AIM_TOOLS_TINYMPC_SOLVER_HPP
#define AIM_TOOLS_TINYMPC_SOLVER_HPP

#include <Eigen/Dense>
#include <cstddef>

// ============================================================================
// 条件编译：TinyMPC 可用性
// ============================================================================
#ifdef TINYMPC_INCLUDE_DIR
  #define AIM_HAS_TINYMPC 1
#else
  // 用户下载 TinyMPC 后取消下面注释，或通过 cmake -DTINYMPC_INCLUDE_DIR=... 传入
  // #define AIM_HAS_TINYMPC 1
#endif

#ifdef AIM_HAS_TINYMPC
  #include "tinympc/types.hpp"
  #include "tinympc/tiny_api.hpp"
#endif

namespace aim {
namespace tools {

/// @brief 单个轴的 TinyMPC 求解器（yaw 或 pitch）
///
/// 内部维护一个 TinySolver 实例。
/// 当 AIM_HAS_TINYMPC 未定义时，回退为存根实现（solve() 返回恒等轨迹）。
class TinyMPCSolver {
public:
  /// @param dt       离散时间步长 (s)
  /// @param horizon  预测时域长度（步数）
  TinyMPCSolver(double dt, int horizon);

  ~TinyMPCSolver();

  // ---- 初始化（仅需调用一次） ----

  /// @brief 设置 MPC 问题
  /// @param max_acc   控制量限幅 (±max_acc)
  /// @param Q_diag    状态权重对角元素 (2x1: [angle_weight, vel_weight])
  /// @param R_diag    控制权重对角元素 (1x1: [acc_weight])
  void setup(double max_acc,
             const Eigen::Vector2d & Q_diag,
             const Eigen::VectorXd & R_diag);

  // ---- 每周期求解 ----

  /// @brief 以 x0 为初态，x_ref 为参考轨迹，求解 MPC
  /// @param x0    当前状态 [angle, angular_velocity]
  /// @param x_ref 参考轨迹 (2 x horizon)
  void solve(const Eigen::Vector2d & x0,
             const Eigen::MatrixXd & x_ref);

  // ---- 结果读取 ----

  /// @brief 读取状态轨迹中第 i 维、第 k 步的值
  double state(int dim, int step) const;

  /// @brief 读取控制轨迹中第 k 步的值（控制维度固定为 1）
  double control(int step) const;

  /// @brief 时域长度
  int horizon() const { return horizon_; }

private:
  int horizon_;
  double dt_;

#ifdef AIM_HAS_TINYMPC
  TinySolver * solver_ = nullptr;
  // 缓存状态和控制轨迹
  Eigen::MatrixXd x_traj_;   // (2 x horizon)
  Eigen::MatrixXd u_traj_;   // (1 x horizon-1)
#endif
};

}  // namespace tools
}  // namespace aim

#endif  // AIM_TOOLS_TINYMPC_SOLVER_HPP