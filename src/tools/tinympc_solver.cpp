/// @file tinympc_solver.cpp — TinyMPC 求解器实现
#include "../../include/tools/tinympc_solver.hpp"
#include <algorithm>

namespace aim {
namespace tools {

// ============================================================================
// AIM_HAS_TINYMPC 路径：链接真实 TinyMPC 库
// ============================================================================
#ifdef AIM_HAS_TINYMPC

// TinyMPC C API（来自 spr_vision_try/tasks/auto_aim/planner/tinympc/tiny_api.hpp）
extern "C" {
int tiny_setup(TinySolver** solverp,
               tinyMatrix Adyn, tinyMatrix Bdyn, tinyMatrix fdyn,
               tinyMatrix Q, tinyMatrix R,
               tinytype rho, int nx, int nu, int N, int verbose);
int tiny_set_bound_constraints(TinySolver* solver,
                               tinyMatrix x_min, tinyMatrix x_max,
                               tinyMatrix u_min, tinyMatrix u_max);
int tiny_set_x0(TinySolver* solver, tinyVector x0);
int tiny_solve(TinySolver* solver);
}

TinyMPCSolver::TinyMPCSolver(double dt, int horizon)
  : horizon_(horizon), dt_(dt), solver_(nullptr) {
  x_traj_.resize(2, horizon_);
  u_traj_.resize(1, horizon_ - 1);
}

TinyMPCSolver::~TinyMPCSolver() {
  // TinySolver 由 TinyMPC 内部管理，这里仅清空指针
  solver_ = nullptr;
}

void TinyMPCSolver::setup(double max_acc,
                          const Eigen::Vector2d & Q_diag,
                          const Eigen::VectorXd & R_diag) {
  // 双积分器模型: A=[1 dt; 0 1], B=[0; dt]
  Eigen::MatrixXd A(2, 2), B(2, 1), f(2, 1);
  A << 1.0, dt_, 0.0, 1.0;
  B << 0.0, dt_;
  f << 0.0, 0.0;

  Eigen::MatrixXd Q = Q_diag.asDiagonal();
  Eigen::MatrixXd R = R_diag.asDiagonal();

  // 初始化求解器
  tiny_setup(&solver_, A, B, f, Q, R, 1.0, 2, 1, horizon_, 0);

  // 状态无界，控制限幅
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

  solver_->work->Xref = x_ref;

  tiny_solve(solver_);

  // 缓存结果
  x_traj_ = solver_->work->x.block(0, 0, 2, horizon_);
  u_traj_ = solver_->work->u.block(0, 0, 1, horizon_ - 1);
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