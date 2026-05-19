/// @file tinympc_solver.cpp — TinyMPC 求解器实现
#include "../../include/tools/tinympc_solver.hpp"
#include <algorithm>

namespace aim {
namespace tools {

// ============================================================================
// AIM_HAS_TINYMPC 路径：链接真实 TinyMPC 库
// ============================================================================
#ifdef AIM_HAS_TINYMPC

TinyMPCSolver::TinyMPCSolver(double dt, int horizon)
  : horizon_(horizon), dt_(dt), solver_(nullptr) {
  x_traj_.resize(2, horizon_);
  u_traj_.resize(1, horizon_ - 1);
}

TinyMPCSolver::~TinyMPCSolver() {
  solver_ = nullptr;
}

void TinyMPCSolver::setup(double max_acc,
                          const Eigen::Vector2d & Q_diag,
                          const Eigen::VectorXd & R_diag) {
  Eigen::MatrixXd A(2, 2), B(2, 1), f(2, 1);
  A << 1.0, dt_, 0.0, 1.0;
  B << 0.0, dt_;
  f << 0.0, 0.0;

  Eigen::MatrixXd Q = Q_diag.asDiagonal();
  Eigen::MatrixXd R = R_diag.asDiagonal();

  tiny_setup(&solver_, A, B, f, Q, R, 1.0, 2, 1, horizon_, 0);

  Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, horizon_, -1e17);
  Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, horizon_,  1e17);
  Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, horizon_ - 1, -max_acc);
  Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, horizon_ - 1,  max_acc);
  tiny_set_bound_constraints(solver_, x_min, x_max, u_min, u_max);

  solver_->settings->max_iter = 10;
}

void TinyMPCSolver::solve(const Eigen::Vector2d & x0,
                          const Eigen::MatrixXd & x_ref) {
  Eigen::VectorXd x0_vec(2);
  x0_vec << x0(0), x0(1);
  tiny_set_x0(solver_, x0_vec);
  tiny_set_x_ref(solver_, x_ref);

  tiny_solve(solver_);

  // 从 solver->solution 读取结果（非 work，work 是 ADMM 内部变量）
  x_traj_ = solver_->solution->x.block(0, 0, 2, horizon_);
  u_traj_ = solver_->solution->u.block(0, 0, 1, horizon_ - 1);
}

double TinyMPCSolver::state(int dim, int step) const {
  if (step < 0 || step >= horizon_) return 0.0;
  return x_traj_(dim, step);
}

double TinyMPCSolver::control(int step) const {
  if (step < 0 || step >= horizon_ - 1) return 0.0;
  return u_traj_(0, step);
}

// ============================================================================
// 存根路径：TinyMPC 不可用
// ============================================================================
#else

TinyMPCSolver::TinyMPCSolver(double dt, int horizon)
  : horizon_(horizon), dt_(dt) {}

TinyMPCSolver::~TinyMPCSolver() = default;

void TinyMPCSolver::setup(double, const Eigen::Vector2d &, const Eigen::VectorXd &) {}

void TinyMPCSolver::solve(const Eigen::Vector2d & x0, const Eigen::MatrixXd & /*x_ref*/) {
  // 存根模式：返回零轨迹（初态保持不动）
  // 实际使用时，链接 TinyMPC 后此路径不会被调用
}

double TinyMPCSolver::state(int, int) const { return 0.0; }
double TinyMPCSolver::control(int) const { return 0.0; }

#endif  // AIM_HAS_TINYMPC

}  // namespace tools
}  // namespace aim