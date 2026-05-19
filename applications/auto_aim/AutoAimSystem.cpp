/// @file AutoAimSystem.cpp
/// @brief System 实现 — 移植自 spr_vision_try 的 target.cpp + tracker.cpp
///
/// 原代码位置对应关系：
///   Target::predict()          → AutoAimSystem::ekf_predict()
///   Target::update()           → AutoAimSystem::ekf_update_ypda()
///   Target::match_armor_id()   → AutoAimSystem::match_armor_id()
///   Target::h_armor_xyz()      → AutoAimSystem::h_armor_xyz()
///   Target::h_jacobian()       → AutoAimSystem::h_jacobian()
///   Target::diverged()         → AutoAimSystem::ekf_diverged()
///   Target::convergened()      → AutoAimSystem::ekf_converged()
///   Tracker::state_machine()   → AutoAimSystem::state_machine()
///   Tracker::set_target()      → AutoAimSystem::set_target()
///   Tracker::update_target()   → AutoAimSystem::update_target()
///   Tracker::track()           → AutoAimSystem::update() (入口)

#include "AutoAimSystem.hpp"

#include "../../include/core/DebugContext.hpp"
#include <opencv2/core/eigen.hpp>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <numeric>

#include "../AutoAimProvider/tools/logger.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

/// 角度归一化到 [-π, π]（原 tools::limit_rad）
inline double limit_rad(double angle) {
  while (angle > kPi)  angle -= kTwoPi;
  while (angle < -kPi) angle += kTwoPi;
  return angle;
}

/// 时间差（秒）（原 tools::delta_time）
inline double delta_time(
  std::chrono::steady_clock::time_point t1,
  std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double>(t1 - t0).count();
}

/// 笛卡尔 → 球坐标（原 tools::xyz2ypd: yaw, pitch, distance）
inline Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz) {
  return {
    std::atan2(xyz.y(), xyz.x()),
    std::atan2(xyz.z(), std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y())),
    xyz.norm()
  };
}

/// 球坐标变换 ypd = f(xyz) 对 xyz 的雅可比（3×3）
inline Eigen::Matrix3d xyz2ypd_jacobian(const Eigen::Vector3d & xyz) {
  double x = xyz.x(), y = xyz.y(), z = xyz.z();
  double r2 = x * x + y * y;
  double r = std::sqrt(r2);
  double d2 = r2 + z * z;
  double d = std::sqrt(d2);
  double dr_dx = x / r, dr_dy = y / r;
  double dd_dx = x / d, dd_dy = y / d, dd_dz = z / d;

  Eigen::Matrix3d H;
  // dyaw/dx, dyaw/dy, dyaw/dz
  H(0, 0) = -y / r2;  H(0, 1) = x / r2;  H(0, 2) = 0;
  // dpitch/dx, dpitch/dy, dpitch/dz
  H(1, 0) = -x * z / (d2 * r);  H(1, 1) = -y * z / (d2 * r);  H(1, 2) = r / d2;
  // ddistance/dx, ddistance/dy, ddistance/dz
  H(2, 0) = dd_dx;  H(2, 1) = dd_dy;  H(2, 2) = dd_dz;
  return H;
}

}  // anonymous namespace

namespace auto_aim {

// ============================================================================
// 常量（原 target.cpp 顶部）
// ============================================================================
constexpr double OUTPOST_ARMOR_Z_STEP_M = 0.1;   // 前哨站装甲板 Z 间距

// ============================================================================
// 构造
// ============================================================================

AutoAimSystem::AutoAimSystem(const std::string & config_path) {
#ifdef HAS_YAML_CPP
  auto yaml = YAML::LoadFile(config_path);
  enemy_color_ = (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;
#else
  // OpenCV FileStorage 回退
  cv::FileStorage fs(config_path, cv::FileStorage::READ);
  if (fs.isOpened()) {
    std::string ec; fs["enemy_color"] >> ec;
    enemy_color_ = (ec == "red") ? Color::red : Color::blue;
    fs["min_detect_count"] >> min_detect_count_;
    fs["max_temp_lost_count"] >> max_temp_lost_count_;
    fs["outpost_max_temp_lost_count"] >> outpost_max_temp_lost_count_;
    normal_temp_lost_count_ = max_temp_lost_count_;
    fs.release();
  } else {
    // 默认值
    enemy_color_ = Color::blue;
    min_detect_count_ = 5;
    max_temp_lost_count_ = 30;
    outpost_max_temp_lost_count_ = 60;
    normal_temp_lost_count_ = 30;
  }
#endif

  // ---- 加载调试用相机标定（EKF 重投影可视化） ----
  try {
    auto yaml2 = YAML::LoadFile(config_path);
    auto cm_data = yaml2["camera_matrix"].as<std::vector<double>>();
    auto dc_data = yaml2["distort_coeffs"].as<std::vector<double>>();
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> cm(cm_data.data());
    Eigen::Matrix<double, 1, 5> dc(dc_data.data());
    cv::eigen2cv(cm, debug_camera_matrix_);
    cv::eigen2cv(dc, debug_distort_coeffs_);

    auto Rcg_data = yaml2["R_camera2gimbal"].as<std::vector<double>>();
    auto tcg_data = yaml2["t_camera2gimbal"].as<std::vector<double>>();
    debug_R_camera2gimbal_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(Rcg_data.data());
    debug_t_camera2gimbal_ = Eigen::Matrix<double, 3, 1>(tcg_data.data());

    auto Rgi_data = yaml2["R_gimbal2imubody"].as<std::vector<double>>();
    debug_R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(Rgi_data.data());
  } catch (const std::exception & e) {
    tools::logger()->warn("[AutoAimSystem] Failed to load debug camera: {}", e.what());
  }

  // 初始化为 LOST 状态
  state_.track_state = TrackState::LOST;
  state_.pre_state   = TrackState::LOST;
  state_.armor_num   = 0;
  state_.ekf_dim     = 0;
  state_.update_count = 0;
  state_.is_converged = false;
  state_.last_timestamp = std::chrono::steady_clock::now();
}

std::string AutoAimSystem::trackStateStr() const {
  switch (state_.track_state) {
    case TrackState::LOST:      return "lost";
    case TrackState::DETECTING: return "detecting";
    case TrackState::TRACKING:  return "tracking";
    case TrackState::TEMP_LOST: return "temp_lost";
    case TrackState::SWITCHING: return "switching";
    default:                    return "unknown";
  }
}

void AutoAimSystem::updateSelfState(const aim::SelfState & self) {
  state_.self = self;
  if (self.valid) {
    Eigen::Quaterniond q(self.imu_qw, self.imu_qx, self.imu_qy, self.imu_qz);
    Eigen::Matrix3d R_ib2ia = q.toRotationMatrix();
    debug_R_gimbal2world_ = debug_R_gimbal2imubody_.transpose() * R_ib2ia * debug_R_gimbal2imubody_;
  }
}

// ============================================================================
// System::update() — 移植自 Tracker::track()
// ============================================================================

void AutoAimSystem::update(const ArmorList & armors_raw) {
  auto t = std::chrono::steady_clock::now();
  auto dt = delta_time(t, state_.last_timestamp);
  state_.last_timestamp = t;

  // 时间间隔过长 → 可能相机离线（原 Tracker::track 逻辑）
  if (state_.track_state != TrackState::LOST && dt > 0.1) {
    state_.track_state = TrackState::LOST;
  }

  // 复制列表以便修改（排序/过滤）
  ArmorList armors = armors_raw;

  // ---- 过滤非我方装甲板（原 Tracker::track 逻辑） ----
  armors.remove_if([&](const Armor & a) { return a.color != enemy_color_; });

  // ---- 按图像中心距离排序（原 Tracker::track 逻辑） ----
  armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);  // TODO: 从配置读取
    return cv::norm(a.center - img_center) < cv::norm(b.center - img_center);
  });

  // ---- 按优先级排序（原 Tracker::track 逻辑） ----
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  // ---- 状态机驱动（原 Tracker::track 逻辑） ----
  bool found;
  if (state_.track_state == TrackState::LOST) {
    found = set_target(armors, t);
  } else {
    found = update_target(armors, t);
  }

  state_.pre_state = state_.track_state;
  state_machine(found);

  // ---- 发散检测（原 Tracker::track 末尾逻辑） ----
  if (state_.track_state != TrackState::LOST && ekf_diverged()) {
    state_.track_state = TrackState::LOST;
    return;
  }

  // ---- 收敛质量检测（原 Tracker::track 末尾逻辑） ----
  if (std::accumulate(ekf_.recent_nis_failures.begin(),
                      ekf_.recent_nis_failures.end(), 0) >=
      static_cast<int>(0.4 * ekf_.window_size)) {
    state_.track_state = TrackState::LOST;
    return;
  }

  // ---- EKF 预测装甲板重投影可视化 ----
  push_ekf_debug_image();

  // 曲线：EKF 状态
  if (state_.armor_num > 0 && ekf_.x.size() >= 8) {
    auto & ctx = aim::DebugContext::getInstance();
    ctx.pushCurveData("ekf_x_m",     static_cast<float>(ekf_.x[0]));
    ctx.pushCurveData("ekf_y_m",     static_cast<float>(ekf_.x[2]));
    ctx.pushCurveData("ekf_vx_ms",   static_cast<float>(ekf_.x[1]));
    ctx.pushCurveData("ekf_vy_ms",   static_cast<float>(ekf_.x[3]));
    ctx.pushCurveData("ekf_angle_deg", static_cast<float>(ekf_.x[6] * 57.3));
    ctx.pushCurveData("ekf_omega_degs", static_cast<float>(ekf_.x[7] * 57.3));
    ctx.pushCurveData("ekf_r_m",     static_cast<float>(ekf_.x[8]));
    ctx.pushCurveData("track_state", static_cast<float>(static_cast<int>(state_.track_state)));
  }
}

// ============================================================================
// EKF 预测（原 Target::predict(dt)）
// ============================================================================

void AutoAimSystem::ekf_predict(double dt) {
  int dim = state_.ekf_dim;
  double a = dt * dt * dt * dt / 4;
  double b = dt * dt * dt / 2;
  double c = dt * dt;

  if (dim == 13) {
    // 13 维状态转移矩阵（前哨站）
    // clang-format off
    Eigen::MatrixXd F(13, 13);
    F << 1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
         0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0,
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0,
         0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1;

    double v1 = 10, v2 = 0.1;
    Eigen::MatrixXd Q(13, 13);
    Q << a*v1, b*v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
         b*v1, c*v1,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0, a*v1, b*v1,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0, b*v1, c*v1,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0, a*v1, b*v1,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0, b*v1, c*v1,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0, a*v2, b*v2, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0, b*v2, c*v2, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0, 5e-4,    0,
            0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0,    0, 5e-4;
    // clang-format on

    auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
      Eigen::VectorXd x_prior = F * x;
      x_prior[6] = limit_rad(x_prior[6]);
      return x_prior;
    };

    // 角速度钳制（原 Target::predict 末尾）
    if (ekf_converged() && state_.target_name == ArmorName::outpost && std::abs(ekf_.x[7]) > 2) {
      ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;
    }

    ekf_.predict(F, Q, f);
    sync_state_from_ekf();
    return;
  }

  // ---- 11 维状态转移矩阵（普通机器人） ----
  // clang-format off
  Eigen::MatrixXd F{
    {1, dt,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  1, dt,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  1,  0,  0,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  1, dt,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  1,  0,  0,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  1, dt,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  1,  0,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  1,  0,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0},
    {0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1}
  };

  double v1, v2;
  if (state_.target_name == ArmorName::outpost) { v1 = 10;  v2 = 0.1; }
  else                                         { v1 = 100; v2 = 400; }

  Eigen::MatrixXd Q{
    {a*v1, b*v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {b*v1, c*v1,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, a*v1, b*v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0, b*v1, c*v1,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, a*v1, b*v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0, b*v1, c*v1,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, a*v2, b*v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0, b*v2, c*v2, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0},
    {     0,      0,      0,      0,      0,      0,      0,      0, 0, 0, 0}
  };
  // clang-format on

  auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
    Eigen::VectorXd x_prior = F * x;
    x_prior[6] = limit_rad(x_prior[6]);
    return x_prior;
  };

  if (ekf_converged() && state_.target_name == ArmorName::outpost && std::abs(ekf_.x[7]) > 2) {
    ekf_.x[7] = ekf_.x[7] > 0 ? 2.51 : -2.51;
  }

  ekf_.predict(F, Q, f);
  sync_state_from_ekf();
}

// ============================================================================
// EKF 观测更新（原 Target::update() → update_ypda()）
// ============================================================================

void AutoAimSystem::ekf_update_ypda(const Armor & armor, int id) {
  // ---- 观测雅可比 ----
  Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

  // ---- 动态观测噪声 ----
  auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
  auto delta_angle = limit_rad(armor.ypr_in_world[0] - center_yaw);
  double r_yaw_scale = 1.0;
  if (state_.target_name == ArmorName::outpost && state_.armor_num == 3 && ekf_.x.size() >= 13) {
    double yaw_resid = std::abs(delta_angle);
    r_yaw_scale += std::min(yaw_resid / 0.15, 3.0);
  }
  Eigen::VectorXd R_dig{
    {4e-3 * r_yaw_scale, 4e-3 * r_yaw_scale,
     std::log(std::abs(delta_angle) + 1) + 1,
     std::log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};
  Eigen::MatrixXd R = R_dig.asDiagonal();

  // ---- 非线性观测函数 ----
  auto h = [&](const Eigen::VectorXd & x) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(x, id);
    Eigen::VectorXd ypd = xyz2ypd(xyz);
    auto angle = limit_rad(x[6] + id * 2 * kPi / state_.armor_num);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  // ---- 观测残差（角度需 wrap） ----
  auto z_subtract = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a - b;
    c[0] = limit_rad(c[0]);
    c[1] = limit_rad(c[1]);
    c[3] = limit_rad(c[3]);
    return c;
  };

  const Eigen::VectorXd & ypd = armor.ypd_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;
  Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

  ekf_.update(z, H, R, h, z_subtract);
  sync_state_from_ekf();
}

// ============================================================================
// 装甲板匹配（原 Target::match_armor_id）
// ============================================================================

int AutoAimSystem::match_armor_id(double z_obs) const {
  double z0 = ekf_.x[4];
  double z1 = ekf_.x[11];
  double z2 = ekf_.x[12];
  double diff0 = std::abs(z_obs - z0);
  double diff1 = std::abs(z_obs - z1);
  double diff2 = std::abs(z_obs - z2);

  if (diff0 <= diff1 && diff0 <= diff2) return 0;
  if (diff1 <= diff0 && diff1 <= diff2) return 1;
  return 2;
}

// ============================================================================
// 装甲板中心 3D 坐标（原 Target::h_armor_xyz）
// ============================================================================

Eigen::Vector3d AutoAimSystem::h_armor_xyz(const Eigen::VectorXd & x, int id) const {
  auto angle = limit_rad(x[6] + id * 2 * kPi / state_.armor_num);
  if (state_.target_name == ArmorName::outpost && state_.armor_num == 3 && static_cast<int>(x.size()) >= 13) {
    auto r = x[8];
    auto armor_x = x[0] - r * std::cos(angle);
    auto armor_y = x[2] - r * std::sin(angle);
    double armor_z = (id == 0) ? x[4] : (id == 1 ? x[11] : x[12]);
    return {armor_x, armor_y, armor_z};
  }
  auto use_l_h = (state_.armor_num == 4) && (id == 1 || id == 3);
  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto armor_x = x[0] - r * std::cos(angle);
  auto armor_y = x[2] - r * std::sin(angle);
  double armor_z = (use_l_h) ? x[4] + x[10] : x[4];
  return {armor_x, armor_y, armor_z};
}

// ============================================================================
// 观测雅可比（原 Target::h_jacobian — 保留原实现）
// ============================================================================

Eigen::MatrixXd AutoAimSystem::h_jacobian(const Eigen::VectorXd & x, int id) const {
  auto angle = limit_rad(x[6] + id * 2 * kPi / state_.armor_num);

  // === 前哨站 13 维解析雅可比 ===
  if (state_.target_name == ArmorName::outpost && state_.armor_num == 3
      && static_cast<int>(x.size()) >= 13) {
    auto r = x[8];
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);
    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    double dz_dz0 = (id == 0) ? 1.0 : 0.0;
    double dz_dz1 = (id == 1) ? 1.0 : 0.0;
    double dz_dz2 = (id == 2) ? 1.0 : 0.0;

    // clang-format off
    Eigen::MatrixXd H_armor_xyza(4, 13);
    H_armor_xyza <<
      1, 0, 0, 0,     0, 0, dx_da, 0, dx_dr, 0, 0,     0,     0,
      0, 0, 1, 0,     0, 0, dy_da, 0, dy_dr, 0, 0,     0,     0,
      0, 0, 0, 0, dz_dz0, 0,     0, 0,     0, 0, 0, dz_dz1, dz_dz2,
      0, 0, 0, 0,     0, 0,     1, 0,     0, 0, 0,     0,     0;
    // clang-format on

    Eigen::Vector3d armor_xyz = h_armor_xyz(x, id);
    Eigen::Matrix3d H_armor_ypd = xyz2ypd_jacobian(armor_xyz);
    Eigen::MatrixXd H_armor_ypda(4, 4);
    // clang-format off
    H_armor_ypda <<
      H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
      H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
      H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
                       0,                  0,                  0, 1;
    // clang-format on
    return H_armor_ypda * H_armor_xyza;
  }

  // === 11 维解析雅可比（普通 / 平衡步兵） ===
  auto use_l_h = (state_.armor_num == 4) && (id == 1 || id == 3);
  auto r = (use_l_h) ? x[8] + x[9] : x[8];
  auto dx_da = r * std::sin(angle);
  auto dy_da = -r * std::cos(angle);
  auto dx_dr = -std::cos(angle);
  auto dy_dr = -std::sin(angle);
  auto dx_dl = (use_l_h) ? -std::cos(angle) : 0.0;
  auto dy_dl = (use_l_h) ? -std::sin(angle) : 0.0;
  auto dz_dh = (use_l_h) ? 1.0 : 0.0;

  // clang-format off
  Eigen::MatrixXd H_armor_xyza(4, 11);
  H_armor_xyza <<
    1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl,     0,
    0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl,     0,
    0, 0, 0, 0, 1, 0,     0, 0,     0,     0, dz_dh,
    0, 0, 0, 0, 0, 0,     1, 0,     0,     0,     0;
  // clang-format on

  Eigen::Vector3d armor_xyz = h_armor_xyz(x, id);
  Eigen::Matrix3d H_armor_ypd = xyz2ypd_jacobian(armor_xyz);
  Eigen::MatrixXd H_armor_ypda(4, 4);
  // clang-format off
  H_armor_ypda <<
    H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0,
    H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0,
    H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0,
                     0,                  0,                  0, 1;
  // clang-format on
  return H_armor_ypda * H_armor_xyza;
}

// ============================================================================
// 发散检测（原 Target::diverged）
// ============================================================================

bool AutoAimSystem::ekf_diverged() const {
  auto r_ok = ekf_.x[8] > 0 && ekf_.x[8] < 0.7;
  auto l_ok = ekf_.x[8] + ekf_.x[9] > 0.07 && ekf_.x[8] + ekf_.x[9] < 0.9;
  return !(r_ok && l_ok);
}

// ============================================================================
// 收敛检测（原 Target::convergened）
// ============================================================================

bool AutoAimSystem::ekf_converged() {
  if (state_.target_name != ArmorName::outpost && state_.update_count > 3 && !ekf_diverged()) {
    state_.is_converged = true;
  }
  if (state_.target_name == ArmorName::outpost && state_.update_count > 50 && !ekf_diverged()) {
    state_.is_converged = true;
  }
  return state_.is_converged;
}

// ============================================================================
// 状态机（原 Tracker::state_machine）
// ============================================================================

void AutoAimSystem::state_machine(bool found) {
  if (state_.track_state == TrackState::LOST) {
    if (!found) return;
    state_.track_state = TrackState::DETECTING;
    state_.detect_count = 1;
  }
  else if (state_.track_state == TrackState::DETECTING) {
    if (found) {
      state_.detect_count++;
      if (state_.detect_count >= min_detect_count_)
        state_.track_state = TrackState::TRACKING;
    } else {
      state_.detect_count = 0;
      state_.track_state = TrackState::LOST;
    }
  }
  else if (state_.track_state == TrackState::TRACKING) {
    if (found) return;
    state_.temp_lost_count = 1;
    state_.track_state = TrackState::TEMP_LOST;
  }
  else if (state_.track_state == TrackState::SWITCHING) {
    if (found) {
      state_.track_state = TrackState::DETECTING;
    } else {
      state_.temp_lost_count++;
      if (state_.temp_lost_count > 200) state_.track_state = TrackState::LOST;
    }
  }
  else if (state_.track_state == TrackState::TEMP_LOST) {
    if (found) {
      state_.track_state = TrackState::TRACKING;
    } else {
      state_.temp_lost_count++;
      if (state_.target_name == ArmorName::outpost)
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;
      if (state_.temp_lost_count > max_temp_lost_count_)
        state_.track_state = TrackState::LOST;
    }
  }
}

// ============================================================================
// 初始化新目标（原 Tracker::set_target）
// ============================================================================

bool AutoAimSystem::set_target(ArmorList & armors, std::chrono::steady_clock::time_point t) {
  if (armors.empty()) return false;

  auto & armor = armors.front();
  // 注意：armor 此时应已包含 xyz_in_world / ypr_in_world / ypd_in_world
  //（由 DataProvider 中的 PnP 阶段填入）

  // 根据兵种选择 EKF 初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  // 旋转中心坐标
  double r = 0.2;  // 默认半径
  int a_num = 4;
  Eigen::VectorXd P0_dig;

  if (is_balance) {
    r = 0.2; a_num = 2;
    P0_dig = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  } else if (armor.name == ArmorName::outpost) {
    r = 0.2765; a_num = 3;
    P0_dig = Eigen::VectorXd{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0, 100, 100}};
  } else if (armor.name == ArmorName::base) {
    r = 0.3205; a_num = 3;
    P0_dig = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
  } else {
    r = 0.2; a_num = 4;
    P0_dig = Eigen::VectorXd{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
  }

  const Eigen::VectorXd & xyz = armor.xyz_in_world;
  const Eigen::VectorXd & ypr = armor.ypr_in_world;

  auto center_x = xyz[0] + r * std::cos(ypr[0]);
  auto center_y = xyz[1] + r * std::sin(ypr[0]);
  auto center_z = xyz[2];

  Eigen::VectorXd x0;
  Eigen::MatrixXd P0;
  int dim = static_cast<int>(P0_dig.size());

  if (a_num == 3 && dim == 13) {
    x0.resize(13);
    x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0,
          center_z + OUTPOST_ARMOR_Z_STEP_M,
          center_z - OUTPOST_ARMOR_Z_STEP_M;
    P0 = P0_dig.asDiagonal();
  } else {
    x0.resize(11);
    x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0;
    P0 = P0_dig.asDiagonal();
  }

  auto x_add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) -> Eigen::VectorXd {
    Eigen::VectorXd c = a + b;
    c[6] = limit_rad(c[6]);
    return c;
  };

  ekf_ = aim::tools::ExtendedKalmanFilter(x0, P0, x_add);

  // 填充 state_ 元数据
  state_.target_name     = armor.name;
  state_.target_type     = armor.type;
  state_.target_priority = armor.priority;
  state_.armor_num       = a_num;
  state_.jumped          = false;
  state_.update_count    = 0;
  state_.is_converged    = false;
  state_.ekf_dim         = dim;
  state_.last_timestamp  = t;

  sync_state_from_ekf();
  return true;
}

// ============================================================================
// 更新已有目标（原 Tracker::update_target）
// ============================================================================

bool AutoAimSystem::update_target(ArmorList & armors, std::chrono::steady_clock::time_point t) {
  ekf_predict(delta_time(t, state_.last_timestamp));
  state_.last_timestamp = t;

  int found_count = 0;
  for (const auto & armor : armors) {
    if (armor.name != state_.target_name || armor.type != state_.target_type) continue;
    found_count++;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (armor.name != state_.target_name || armor.type != state_.target_type) continue;

    // armor 的 3D 坐标由 DataProvider 中的 PnP 阶段填入
    // 这里直接使用进行 EKF 更新

    // ---- 装甲板匹配（原 Target::update 逻辑） ----
    int id = 0;
    if (state_.target_name == ArmorName::outpost && state_.armor_num == 3 && state_.ekf_dim >= 13) {
      double z0_var = ekf_.P(4, 4);
      double z1_var = ekf_.P(11, 11);
      double z2_var = ekf_.P(12, 12);
      bool z_converged = (z0_var < 0.01) && (z1_var < 0.01) && (z2_var < 0.01) &&
                         (std::abs(ekf_.x[11] - ekf_.x[4]) > 0.03 ||
                          std::abs(ekf_.x[12] - ekf_.x[4]) > 0.03);

      double z_obs = armor.xyz_in_world[2];
      int height_matched_id = match_armor_id(z_obs);
      double min_error = 1e10;
      int final_id = height_matched_id;

      for (int offset = -1; offset <= 1; offset++) {
        int check_id = (height_matched_id + offset + state_.armor_num) % state_.armor_num;
        auto check_angle = limit_rad(ekf_.x[6] + check_id * 2 * kPi / state_.armor_num);
        double angle_error = std::abs(limit_rad(armor.ypr_in_world[0] - check_angle));
        auto check_xyz = h_armor_xyz(ekf_.x, check_id);
        double height_error = std::abs(z_obs - check_xyz[2]) * (z_converged ? 2.0 : 0.5);
        double total_error = angle_error + height_error;

        if (total_error < min_error) {
          min_error = total_error;
          final_id = check_id;
        }
      }
      id = final_id;
    } else {
      // 距离排序 + 角度&ypd组合匹配（原 Target::update 逻辑）
      auto min_angle_error = 1e10;
      std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
      for (int i = 0; i < state_.armor_num; i++) {
        auto angle = limit_rad(ekf_.x[6] + i * 2 * kPi / state_.armor_num);
        auto xyz = h_armor_xyz(ekf_.x, i);
        xyza_i_list.push_back({{xyz[0], xyz[1], xyz[2], angle}, i});
      }

      // 按距离排序取前 3 个
      std::sort(xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int> & a,
           const std::pair<Eigen::Vector4d, int> & b) {
          Eigen::Vector3d ypd1 = xyz2ypd(a.first.head<3>());
          Eigen::Vector3d ypd2 = xyz2ypd(b.first.head<3>());
          return ypd1[2] < ypd2[2];
        });

      for (int i = 0; i < 3 && i < static_cast<int>(xyza_i_list.size()); i++) {
        const auto & xyza = xyza_i_list[i].first;
        Eigen::Vector3d ypd = xyz2ypd(xyza.head<3>());
        auto angle_error = std::abs(limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                           std::abs(limit_rad(armor.ypd_in_world[0] - ypd[0]));
        if (std::abs(angle_error) < std::abs(min_angle_error)) {
          id = xyza_i_list[i].second;
          min_angle_error = angle_error;
        }
      }
    }

    if (id != 0) state_.jumped = true;

    // ---- EKF 更新 ----
    state_.update_count++;
    ekf_update_ypda(armor, id);
  }

  sync_state_from_ekf();
  return true;
}

// ============================================================================
// 同步 EKF → state_
// ============================================================================

void AutoAimSystem::sync_state_from_ekf() {
  state_.ekf_x = ekf_.x;
  state_.ekf_P = ekf_.P;
  state_.ekf_dim = static_cast<int>(ekf_.x.size());
}

// ============================================================================
// EKF 调试（全部 N 块装甲板重投影）
// ============================================================================

std::vector<Eigen::Vector4d> AutoAimSystem::ekf_armor_xyza_list() const {
  std::vector<Eigen::Vector4d> list;
  if (state_.armor_num == 0) return list;
  for (int i = 0; i < state_.armor_num; i++) {
    auto angle = limit_rad(ekf_.x[6] + i * 2 * kPi / state_.armor_num);
    auto xyz = h_armor_xyz(ekf_.x, i);
    list.push_back({xyz[0], xyz[1], xyz[2], angle});
  }
  return list;
}

void AutoAimSystem::push_ekf_debug_image() {
  if (debug_camera_matrix_.empty() || state_.armor_num == 0) return;

  // 从 DebugContext 取原始相机帧
  auto images = aim::DebugContext::getInstance().getImages();
  auto it = images.find("Camera");
  if (it == images.end() || it->second.empty()) return;

  cv::Mat ekf_img = it->second.clone();
  if (ekf_img.empty()) return;

  const std::vector<cv::Point3f> BIG_PTS{
    {0,  0.115,  0.028}, {0, -0.115,  0.028},
    {0, -0.115, -0.028}, {0,  0.115, -0.028}};
  const std::vector<cv::Point3f> SML_PTS{
    {0,  0.0675,  0.028}, {0, -0.0675,  0.028},
    {0, -0.0675, -0.028}, {0,  0.0675, -0.028}};

  auto & R_cg = debug_R_camera2gimbal_;
  auto & t_cg = debug_t_camera2gimbal_;
  auto & R_gw = debug_R_gimbal2world_;
  auto & K   = debug_camera_matrix_;
  auto & D   = debug_distort_coeffs_;

  auto plates = ekf_armor_xyza_list();
  for (const auto & plate : plates) {
    double yaw = plate[3];
    Eigen::Vector3d xyz_world(plate[0], plate[1], plate[2]);
    auto type = state_.target_type;

    const auto & pts = (type == ArmorType::big) ? BIG_PTS : SML_PTS;

    double sin_yaw = std::sin(yaw), cos_yaw = std::cos(yaw);
    double pitch = 15.0 * CV_PI / 180.0;
    double sin_p = std::sin(pitch), cos_p = std::cos(pitch);
    Eigen::Matrix3d R_aw{
      {cos_yaw * cos_p, -sin_yaw, cos_yaw * sin_p},
      {sin_yaw * cos_p,  cos_yaw, sin_yaw * sin_p},
      {        -sin_p,        0,           cos_p}};

    Eigen::Matrix3d R_ac = R_cg.transpose() * R_gw.transpose() * R_aw;
    Eigen::Vector3d t_ac = R_cg.transpose() * (R_gw.transpose() * xyz_world - t_cg);

    cv::Mat R_ac_cv; cv::eigen2cv(R_ac, R_ac_cv);
    cv::Vec3d rvec;  cv::Rodrigues(R_ac_cv, rvec);
    cv::Vec3d tvec(t_ac[0], t_ac[1], t_ac[2]);

    std::vector<cv::Point2f> img_pts;
    cv::projectPoints(pts, rvec, tvec, K, D, img_pts);

    // 红色 = EKF 预测的全部装甲板
    for (const auto & pt : img_pts)
      cv::circle(ekf_img, pt, 3, {0, 0, 255}, -1);
    for (size_t i = 0; i < img_pts.size(); i++)
      cv::line(ekf_img, img_pts[i], img_pts[(i + 1) % img_pts.size()], {0, 0, 255}, 1);
  }

  aim::DebugContext::getInstance().setImage("EKF", ekf_img);
}

}  // namespace auto_aim