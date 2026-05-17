#ifndef AIM_FRAMEWORK_CORE_TYPES_HPP
#define AIM_FRAMEWORK_CORE_TYPES_HPP

#include <Eigen/Core>
#include <string>
#include <chrono>
#include <optional>

namespace aim {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
using Duration = std::chrono::duration<double>;

// 统一的物理目标状态表示
struct TargetState {
    TimePoint timestamp;
    std::string frame_id;
    bool valid = false;

    // Image plane observation (for 2D trackers)
    bool has_image_point = false;
    Vec2 image_point = Vec2::Zero();

    // 3D Position
    bool has_position = false;
    Vec3 position = Vec3::Zero();

    // 3D Velocity
    bool has_velocity = false;
    Vec3 velocity = Vec3::Zero();

    // 3D Acceleration
    bool has_acceleration = false;
    Vec3 acceleration = Vec3::Zero();
    
    // Euler angles (e.g. for vehicles)
    bool has_euler = false;
    Vec3 euler = Vec3::Zero(); // roll, pitch, yaw
};

// 统一的控制指令表示基类或通用结构
struct GimbalCommand {
    double yaw = 0.0;
    double pitch = 0.0;
    
    // 增量控制 (对于没有绝对坐标反馈或纯图像跟踪很有用)
    double yaw_delta = 0.0;
    double pitch_delta = 0.0;

    // 速度控制 (IBVS 视觉伺服常用)
    double yaw_vel = 0.0;
    double pitch_vel = 0.0;
    
    bool is_absolute = false; // 是否是绝对角度控制
};

} // namespace aim

#endif // AIM_FRAMEWORK_CORE_TYPES_HPP
