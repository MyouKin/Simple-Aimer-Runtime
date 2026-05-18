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

// ---------- 数学工具 — 从 spr_vision_try/tools/math_tools.hpp 移植关键函数 ----------
// 完整版在 spr_vision_try 中，这里只移植 EKF 所需的三个函数
#include <cmath>
#include <numeric>

// 配置读取：优先 yaml-cpp，回退 OpenCV FileStorage
#ifdef HAS_YAML_CPP
#include <yaml-cpp/yaml.h>
#else
#include <opencv2/opencv.hpp>
#endif

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

  // 初始化为 LOST 状态
  state_.track_state = TrackState::LOST;
  state_.pre_state   = TrackState::LOST;
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
  // 原代码中的 h_jacobian 使用数值微分或解析雅可比
  // 这里保留与原代码相同的结构：用中心差分近似雅可比
  const double eps = 1e-6;
  int dim = static_cast<int>(x.size());
  int obs_dim = 4;
  Eigen::MatrixXd H(obs_dim, dim);

  auto h = [&](const Eigen::VectorXd & xx) -> Eigen::Vector4d {
    Eigen::VectorXd xyz = h_armor_xyz(xx, id);
    Eigen::VectorXd ypd = xyz2ypd(xyz);
    auto angle = limit_rad(xx[6] + id * 2 * kPi / state_.armor_num);
    return {ypd[0], ypd[1], ypd[2], angle};
  };

  Eigen::VectorXd h0 = h(x);
  for (int j = 0; j < dim; ++j) {
    Eigen::VectorXd xp = x, xn = x;
    xp[j] += eps;
    xn[j] -= eps;
    Eigen::VectorXd hp = h(xp);
    Eigen::VectorXd hn = h(xn);
    for (int i = 0; i < obs_dim; ++i) {
      double diff = hp[i] - hn[i];
      if (i == 0 || i == 3) diff = limit_rad(diff);  // 角度维度 wrap
      H(i, j) = diff / (2.0 * eps);
    }
  }
  return H;
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
      double z_obs = armor.xyz_in_world[2];
      int height_matched_id = match_armor_id(z_obs);
      id = height_matched_id;  // 简化：直接用高度匹配结果
    } else {
      // 角度最近匹配
      double min_err = 1e10;
      for (int i = 0; i < state_.armor_num; ++i) {
        auto angle = limit_rad(ekf_.x[6] + i * 2 * kPi / state_.armor_num);
        double err = std::abs(limit_rad(armor.ypr_in_world[0] - angle));
        if (err < min_err) { min_err = err; id = i; }
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

}  // namespace auto_aim