/// @file main.cpp
/// @brief AutoAim 应用入口
///
/// 管道：AutoAimProvider → AutoAimSystem → AutoAimSelector → AutoAimSolver → AutoAimGimbal
///
/// Actuator 内部使用 io::Gimbal（串口收发），后台线程持续读取云台反馈。
/// Runtime 每帧调用 actuator->feedback() 获取最新 IMU 四元数，
/// Provider 通过 acceptFeedback() 更新 PnP 世界变换。

#include "../../include/runtime/Runtime.hpp"
#include "GimbalActuator.hpp"
#include "AutoAimProvider/AutoAimProvider.hpp"
#include "AutoAimSystem.hpp"
#include "AutoAimSelector.hpp"
#include "AutoAimSolver.hpp"

#include "AutoAimProvider/io/gimbal/gimbal.hpp"

#include <iostream>
#include <memory>

using namespace auto_aim;

// ============================================================================
// AutoAimGimbal — 封装 io::Gimbal 实现串口收发
// ============================================================================

class AutoAimGimbal : public aim::GimbalActuator {
public:
  explicit AutoAimGimbal(const std::string & config_path)
    : gimbal_(config_path)
  {
    setControlMode(aim::GimbalActuator::ControlMode::FULL);
  }

  /// @brief 获取最新云台状态（非阻塞，由内部 read_thread 持续更新）
  io::GimbalState gimbalState() const { return gimbal_.state(); }

  /// @brief 获取时刻 t 的 IMU 四元数（球面线性插值）
  Eigen::Quaterniond imuQuaternion(std::chrono::steady_clock::time_point t) {
    return gimbal_.q(t);
  }

protected:
  bool transmit(const aim::Command & cmd) override {
    gimbal_.send(
      true,                         // control
      cmd.is_fine_aiming,           // fire
      static_cast<float>(cmd.yaw),
      static_cast<float>(cmd.yaw_vel),
      static_cast<float>(cmd.yaw_acc),
      static_cast<float>(cmd.pitch),
      static_cast<float>(cmd.pitch_vel),
      static_cast<float>(cmd.pitch_acc));
    return true;
  }

  std::optional<aim::SelfState> receive() override {
    auto t = std::chrono::steady_clock::now();
    auto gs = gimbal_.state();
    auto q  = gimbal_.q(t);

    aim::SelfState s;
    s.timestamp    = aim::TimePoint(std::chrono::duration_cast<std::chrono::nanoseconds>(
                      t.time_since_epoch()));
    s.yaw          = static_cast<double>(gs.yaw);
    s.pitch        = static_cast<double>(gs.pitch);
    s.yaw_vel      = static_cast<double>(gs.yaw_vel);
    s.pitch_vel    = static_cast<double>(gs.pitch_vel);
    s.imu_qw       = static_cast<double>(q.w());
    s.imu_qx       = static_cast<double>(q.x());
    s.imu_qy       = static_cast<double>(q.y());
    s.imu_qz       = static_cast<double>(q.z());
    s.bullet_speed = static_cast<double>(gs.bullet_speed);
    s.bullet_count = gs.bullet_count;
    s.valid        = true;
    return s;
  }

private:
  io::Gimbal gimbal_;
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