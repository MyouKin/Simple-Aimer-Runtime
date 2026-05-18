#ifndef AUTO_AIM__TYPES_HPP
#define AUTO_AIM__TYPES_HPP
/// @file types.hpp
/// @brief AutoAim 任务的全部类型定义
///
/// 从 spr_vision_try/tasks/auto_aim/armor.hpp 完整移植：
///   - Color / ArmorType / ArmorName / ArmorPriority 枚举
///   - Lightbar / Armor 结构体
///   - armor_properties 静态表
///
/// 新增（适配 Simple-Aimer-Runtime）：
///   - AutoAimSystemState : SystemStateType（EKF 状态 + 追踪器元数据 + 云台状态）

#include "../../include/core/types.hpp"   // aim::FinalTargetState, aim::Command
#include "../../include/pipeline/System.hpp" // aim::SelfState

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

// ============================================================================
// 1. 枚举定义（原封不动保留）
// ============================================================================

namespace auto_aim {

enum Color {
  red,
  blue,
  extinguish,
  purple
};
const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType {
  big,
  small
};
const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName {
  one,
  two,
  three,
  four,
  five,
  sentry,
  outpost,
  base,
  not_armor
};
const std::vector<std::string> ARMOR_NAMES = {"one",    "two",     "three", "four",     "five",
                                              "sentry", "outpost", "base",  "not_armor"};

enum ArmorPriority {
  first = 1,
  second,
  third,
  forth,
  fifth
};

// ============================================================================
// 2. ARMOR_PROPERTIES 静态表（原封不动保留）
// ============================================================================

// clang-format off
const std::vector<std::tuple<Color, ArmorName, ArmorType>> armor_properties = {
  {blue, sentry, small},     {red, sentry, small},     {extinguish, sentry, small},
  {blue, one, small},        {red, one, small},        {extinguish, one, small},
  {blue, two, small},        {red, two, small},        {extinguish, two, small},
  {blue, three, small},      {red, three, small},      {extinguish, three, small},
  {blue, four, small},       {red, four, small},       {extinguish, four, small},
  {blue, five, small},       {red, five, small},       {extinguish, five, small},
  {blue, outpost, small},    {red, outpost, small},    {extinguish, outpost, small},
  {blue, base, big},         {red, base, big},         {extinguish, base, big},      {purple, base, big},
  {blue, base, small},       {red, base, small},       {extinguish, base, small},    {purple, base, small},
  {blue, three, big},        {red, three, big},        {extinguish, three, big},
  {blue, four, big},         {red, four, big},         {extinguish, four, big},
  {blue, five, big},         {red, five, big},         {extinguish, five, big}};
// clang-format on

// ============================================================================
// 3. Lightbar / Armor 结构体（原封不动保留）
// ============================================================================

struct Lightbar {
  std::size_t id;
  Color color;
  cv::Point2f center, top, bottom, top2bottom;
  std::vector<cv::Point2f> points;
  double angle, angle_error, length, width, ratio;
  cv::RotatedRect rotated_rect;

  Lightbar(const cv::RotatedRect & rotated_rect, std::size_t id);
  Lightbar() {};
};

struct Armor {
  Color color;
  Lightbar left, right;
  cv::Point2f center;       // 不是对角线交点，不能作为实际中心！
  cv::Point2f center_norm;  // 归一化坐标
  std::vector<cv::Point2f> points;

  double ratio;              // 两灯条中点连线与长灯条长度之比
  double side_ratio;         // 长灯条与短灯条长度之比
  double rectangular_error;  // 灯条和中点连线所成夹角与π/2 的差值

  ArmorType type;
  ArmorName name;
  ArmorPriority priority;
  int class_id;
  cv::Rect box;
  cv::Mat pattern;
  double confidence;
  bool duplicated;

  Eigen::Vector3d xyz_in_gimbal;  // 单位：m
  Eigen::Vector3d xyz_in_world;   // 单位：m
  Eigen::Vector3d ypr_in_gimbal;  // 单位：rad
  Eigen::Vector3d ypr_in_world;   // 单位：rad
  Eigen::Vector3d ypd_in_world;   // 球坐标系

  double yaw_raw;  // rad

  Armor(const Lightbar & left, const Lightbar & right);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int class_id, float confidence, const cv::Rect & box, std::vector<cv::Point2f> armor_keypoints,
    cv::Point2f offset);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints);
  Armor(
    int color_id, int num_id, float confidence, const cv::Rect & box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

// ============================================================================
// 4. SystemStateType — 适配 Simple-Aimer-Runtime（新增）
// ============================================================================

/// @brief 追踪器状态机状态（完全对应原 Tracker 的 state_）
enum class TrackState {
  LOST,
  DETECTING,
  TRACKING,
  TEMP_LOST,
  SWITCHING
};

/// @brief AutoAim 系统的全量状态
///
/// 作为 Runtime 的 SystemStateType 使用。
/// 包含：
///   - EKF 状态（位置/速度/旋转/几何） — 来自原 Target 类
///   - 追踪器状态机元数据 — 来自原 Tracker 类
///   - 云台反馈状态 — 来自 Actuator::feedback() → System::updateGimbalState()
struct AutoAimSystemState {
  // ---- EKF 核心状态（来自原 Target::ekf_） ----
  // 11维：[x, vx, y, vy, z, vz, a, ω, r, l, h]
  // 13维（前哨站）：上面 + [z1, z2]
  Eigen::VectorXd ekf_x;       // EKF 状态均值
  Eigen::MatrixXd ekf_P;       // EKF 状态协方差
  int ekf_dim;                 // 实际维度 (11 或 13)

  // ---- 目标元数据（来自原 Target 字段） ----
  ArmorName target_name     = ArmorName::not_armor;
  ArmorType target_type     = ArmorType::small;
  ArmorPriority target_priority = ArmorPriority::fifth;
  int armor_num             = 4;   // 装甲板数量 (2/3/4)
  bool jumped               = false;
  int update_count          = 0;
  bool is_converged         = false;

  // ---- 追踪器状态机（来自原 Tracker 字段） ----
  TrackState track_state    = TrackState::LOST;
  int detect_count          = 0;
  int temp_lost_count       = 0;
  TrackState pre_state      = TrackState::LOST;  // 前一个状态（用于 switching 判断）

  // ---- 时间戳 ----
  std::chrono::steady_clock::time_point last_timestamp;

  // ---- 自身状态（由 Actuator 反馈，System::updateSelfState 写入） ----
  aim::SelfState self;

  // ---- 快捷判断 ----
  bool has_valid_target() const {
    return track_state == TrackState::TRACKING ||
           track_state == TrackState::TEMP_LOST;
  }
};

// ============================================================================
// 5. 管线类型别名（适配 Simple-Aimer-Runtime 模板参数）
// ============================================================================

/// DataProvider 输出类型：检测到的装甲板列表（含 3D 世界坐标）
using ArmorList = std::list<Armor>;

}  // namespace auto_aim

#endif  // AUTO_AIM__TYPES_HPP