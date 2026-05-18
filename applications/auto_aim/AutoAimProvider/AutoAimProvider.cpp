/// @file AutoAimProvider.cpp
/// @brief DataProvider 实现 — 相机采集 + 传统检测 + ImGui 调试图像推送
///
/// ## 数据流（对应 spr_vision_try main 线程前半段）
///
/// ```text
/// Camera.read(img, timestamp)
///     │
///     ├── DebugContext::setImage("Camera", img)   ← ImGui 显示
///     │
///     └── Detector::detect(img) → ArmorList
///             │
///             └── 每个 Armor 含 name/type/priority/points
///                 （xyz_in_world 等 3D 字段待 PnP 阶段填入）
/// ```

#include "AutoAimProvider.hpp"

#include "../../include/core/DebugContext.hpp"
#include "io/camera.hpp"

#include <iostream>

namespace auto_aim {

AutoAimProvider::AutoAimProvider(const std::string & config_path) {
  // 1. 创建相机实例（工厂根据 camera_name 选驱动）
  std::cout << "[AutoAimProvider] Initializing camera from config: " << config_path << std::endl;
  camera_ = std::make_unique<io::Camera>(config_path);

  // 2. 创建检测器（传统灯条检测）
  std::cout << "[AutoAimProvider] Initializing Detector from config: " << config_path << std::endl;
  detector_ = std::make_unique<Detector>(config_path, false);
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

  // 3. 传统检测 → ArmorList
  out_armors = detector_->detect(last_frame_);

  // 4. 绘制检测结果图像 → 推送到 ImGui 调试面板
  auto detection_img = draw_detection(last_frame_, out_armors);
  aim::DebugContext::getInstance().setImage("Detection", detection_img);

  // 5. TODO: PnP 3D 解算（填入 xyz_in_world / ypr_in_world / ypd_in_world）
  //          当前阶段只做 2D 检测，3D 字段均为零

  return true;
}

}  // namespace auto_aim
