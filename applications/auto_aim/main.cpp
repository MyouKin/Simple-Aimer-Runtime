/// @file main.cpp
/// @brief AutoAim 应用入口
///
/// 管道：AutoAimProvider → AutoAimSystem → AutoAimSelector → AutoAimSolver → AutoAimGimbal
///                                                                              ↑
///                                                         继承自 aim::GimbalActuator
///                                                         覆写 transmit() 实现串口协议
///                                                         覆写 receive()  实现回读反馈

#include "../../include/runtime/Runtime.hpp"
#include "GimbalActuator.hpp"
#include "AutoAimProvider.hpp"
#include "AutoAimSystem.hpp"
#include "AutoAimSelector.hpp"
#include "AutoAimSolver.hpp"

#include <iostream>
#include <memory>

using namespace auto_aim;

// ============================================================================
// AutoAimGimbal — 继承运行时的 GimbalActuator，仅需实现串口收发
// ============================================================================

/// @brief auto_aim 专用云台执行器
///
/// 使用 VisionToGimbal / GimbalToVision 协议帧格式。
/// 将 is_fine_aiming 标志解析为开火指令。
class AutoAimGimbal : public aim::GimbalActuator {
public:
  explicit AutoAimGimbal(const std::string & config_path) {
    // TODO: 从 config 读取串口参数 (port, baud_rate)，初始化串口连接
    setControlMode(aim::GimbalActuator::ControlMode::FULL);  // MPC 需要全部控制量
  }

protected:
  bool transmit(const aim::Command & cmd) override {
    // TODO: 打包为 VisionToGimbal 帧并通过串口发送
    //
    // VisionToGimbal 帧格式 (参考 spr_vision_try/io/gimbal/gimbal.hpp):
    //   head[2] = {'S','P'}
    //   mode     = cmd.is_fine_aiming ? 2 : 1   // 2=控制且开火, 1=仅控制
    //   yaw      = float(cmd.yaw)
    //   yaw_vel  = float(cmd.yaw_vel)
    //   yaw_acc  = float(cmd.yaw_acc)
    //   pitch    = float(cmd.pitch)
    //   pitch_vel = float(cmd.pitch_vel)
    //   pitch_acc = float(cmd.pitch_acc)
    //   tail     = 0xef
    //
    // serial.write(packet, sizeof(packet));
    return true;
  }

  std::optional<aim::SelfState> receive() override {
    // TODO: 从串口读取 GimbalToVision 帧，解析为 SelfState
    //
    // GimbalToVision 帧格式:
    //   head[2] = {'S','P'}
    //   q[4]    = IMU 四元数 (wxyz)
    //   yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count
    //   tail    = 0xef
    //
    // aim::SelfState gs;
    // gs.yaw      = rx.yaw;
    // gs.yaw_vel  = rx.yaw_vel;
    // gs.pitch    = rx.pitch;
    // gs.pitch_vel = rx.pitch_vel;
    // gs.timestamp = std::chrono::high_resolution_clock::now();
    // gs.valid     = true;
    // return gs;
    return std::nullopt;  // 暂未实现回读
  }
};

// ============================================================================
// main
// ============================================================================

int main(int argc, char * argv[]) {
  std::string config_path = "configs/standard4.yaml";
  if (argc > 1) config_path = argv[1];

  std::cout << "[AutoAim] Loading config: " << config_path << std::endl;
  std::cout << "[AutoAim] Starting Runtime pipeline..." << std::endl;

  auto provider  = std::make_shared<AutoAimProvider>(config_path);
  auto system    = std::make_shared<AutoAimSystem>(config_path);
  auto selector  = std::make_shared<AutoAimSelector>();
  auto solver    = std::make_shared<AutoAimSolver>(config_path);
  auto actuator  = std::make_shared<AutoAimGimbal>(config_path);

  aim::Runtime<ArmorList, AutoAimSystemState> runtime(
    provider, system, selector, solver, actuator, 100.0);

  runtime.start();
  std::cout << "[AutoAim] Pipeline running. Press Ctrl+C to stop." << std::endl;
  runtime.runUI();
  runtime.stop();
  std::cout << "[AutoAim] Exiting." << std::endl;

  return 0;
}