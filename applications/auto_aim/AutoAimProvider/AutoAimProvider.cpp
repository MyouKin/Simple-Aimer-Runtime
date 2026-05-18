/// @file AutoAimProvider.cpp
/// @brief DataProvider 实现 — 相机采集 + 检测（神经网络/传统） + PnP + ImGui 推送
///
/// ## 数据流
///
/// ```text
/// Camera.read(img)
///     │
///     ├── DebugContext("Camera", img)            ← ImGui 原始画面
///     │
///     └── if use_traditional == false (默认):
///             YOLO::detect(img) → ArmorList      ← 神经网络
///         else:
///             Detector::detect(img) → ArmorList   ← 传统 HSV+轮廓
///         │
///         ├── for each armor: PnPSolver::solve() ← PnP 3D 解算
///         │
///         └── draw_detection(img, armors) → DebugContext("Detection")
/// ```

#include "AutoAimProvider.hpp"

#include "../../include/core/DebugContext.hpp"
#include "io/camera.hpp"
#include "tools/yaml.hpp"

#include <iostream>

namespace auto_aim {

AutoAimProvider::AutoAimProvider(const std::string & config_path) {
  // 1. 创建相机实例（工厂根据 camera_name 选驱动）
  std::cout << "[AutoAimProvider] Initializing camera from config: " << config_path << std::endl;
  camera_ = std::make_unique<io::Camera>(config_path);

  // 2. 读取 use_traditional 决定检测方式
  auto config = tools::load(config_path);
  bool use_traditional = config["use_traditional"]
    ? config["use_traditional"].as<bool>() : false;

  if (!use_traditional) {
    // 神经网络模式（默认）
    std::cout << "[AutoAimProvider] Initializing YOLO (neural network) from config: "
              << config_path << std::endl;
    yolo_ = std::make_unique<YOLO>(config_path, false);
  } else {
    // 传统方法模式
    std::cout << "[AutoAimProvider] Initializing Detector (traditional) from config: "
              << config_path << std::endl;
    detector_ = std::make_unique<Detector>(config_path, false);
  }

  // 3. 创建 PnP 解算器（读取相机内参 + 手眼标定参数）
  std::cout << "[AutoAimProvider] Initializing PnPSolver from config: " << config_path << std::endl;
  pnp_solver_ = std::make_unique<PnPSolver>(config_path);
}

AutoAimProvider::~AutoAimProvider() = default;

bool AutoAimProvider::fetch(ArmorList & out_armors) {
  // 1. 从相机读取一帧
  camera_->read(last_frame_, last_timestamp_);
  if (last_frame_.empty()) {
    return false;
  }

  // 2. 推送图像到 ImGui 调试面板
  aim::DebugContext::getInstance().setImage("Camera", last_frame_);

  // 3. 检测（神经网络 / 传统方法）
  if (yolo_) {
    out_armors = yolo_->detect(last_frame_);
  } else if (detector_) {
    out_armors = detector_->detect(last_frame_);
  }

  // 4. PnP 3D 解算（填入 xyz_in_world / ypr_in_world / ypd_in_world）
  for (auto & armor : out_armors) {
    pnp_solver_->solve(armor);
  }

  // 5. 绘制检测结果图像 → 推送到 ImGui 调试面板
  auto detection_img = draw_detection(last_frame_, out_armors);

  // 6. 在检测图像上叠加重投影点（蓝色 = 优化后的 3D→2D 投影）
  for (const auto & armor : out_armors) {
    if (armor.ypd_in_world.norm() < 1e-6) continue;  // PnP 未成功
    auto reproj = pnp_solver_->reproject_armor(
      armor.xyz_in_world, armor.ypr_in_world[0], armor.type, armor.name);
    for (const auto & pt : reproj) {
      cv::circle(detection_img, pt, 3, {255, 0, 0}, -1);  // 蓝色实心圆
    }
    // 连线
    for (size_t i = 0; i < reproj.size(); i++) {
      cv::line(detection_img, reproj[i], reproj[(i + 1) % reproj.size()], {255, 0, 0}, 1);
    }
  }

  aim::DebugContext::getInstance().setImage("Detection", detection_img);

  return true;
}

}  // namespace auto_aim
