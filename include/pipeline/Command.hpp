/// @file Command.hpp
/// @brief 控制指令 — 唯一指令类型，Solver 选择性填充，Actuator 自定义解析
#ifndef AIM_FRAMEWORK_CORE_COMMAND_HPP
#define AIM_FRAMEWORK_CORE_COMMAND_HPP

namespace aim {

/// @brief 通用云台控制指令
///
/// Solver 按照自身的控制逻辑选择性填充以下字段：
///   - 角度控制: yaw, pitch
///   - 速度控制: yaw_vel, pitch_vel
///   - 加速度前馈: yaw_acc, pitch_acc (MPC 输出)
///
/// Actuator 按硬件协议自定义解析。
/// is_fine_aiming 由应用层按需解读（auto_aim → 开火）。
struct Command {
    double yaw = 0.0;
    double pitch = 0.0;
    double yaw_delta = 0.0;
    double pitch_delta = 0.0;
    double yaw_vel = 0.0;
    double pitch_vel = 0.0;
    double yaw_acc = 0.0;
    double pitch_acc = 0.0;
    bool is_absolute = false;
    bool is_fine_aiming = false;
};

} // namespace aim

#endif