/// @file FinalTargetState.hpp
/// @brief Selector 输出的标准化目标状态 — 框架中间格式
#ifndef AIM_FRAMEWORK_CORE_FINAL_TARGET_STATE_HPP
#define AIM_FRAMEWORK_CORE_FINAL_TARGET_STATE_HPP

#include <Eigen/Core>
#include <string>
#include <chrono>

namespace aim {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
using Duration = std::chrono::duration<double>;

/// @brief 最终目标状态（框架标准中间格式）
///
/// 由 Selector 从 SystemState.TargetState（用户自定义建模）中提取，
/// 传递给 Solver 用于控制解算。
struct FinalTargetState {
    TimePoint timestamp;
    std::string frame_id;
    bool valid = false;

    bool has_image_point = false;
    Vec2 image_point = Vec2::Zero();

    bool has_position = false;
    Vec3 position = Vec3::Zero();

    bool has_velocity = false;
    Vec3 velocity = Vec3::Zero();

    bool has_acceleration = false;
    Vec3 acceleration = Vec3::Zero();

    bool has_euler = false;
    Vec3 euler = Vec3::Zero();
};

} // namespace aim

#endif