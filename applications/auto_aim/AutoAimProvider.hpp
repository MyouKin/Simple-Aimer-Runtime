#ifndef AUTO_AIM__PROVIDER_HPP
#define AUTO_AIM__PROVIDER_HPP
/// @file AutoAimProvider.hpp
/// @brief 数据提供层 — 相机采集 + 装甲板检测 + PnP 3D解算
///
/// ## 实现指南
///
/// 本 Provider 需要集成以下三个组件（由用户根据实际硬件实现）：
///
///   1. 相机采集    — 从工业相机/USB相机读取图像帧
///      参考：spr_vision_try/io/camera.hpp (MindVision SDK) 或 OpenCV VideoCapture
///
///   2. YOLO 检测   — 在图像中检测装甲板，输出 keypoints + class_id
///      参考：spr_vision_try/tasks/auto_aim/yolo.hpp
///      模型权重放在 applications/auto_aim/models/ 目录下
///
///   3. PnP 3D解算  — 将 2D keypoints 转为 3D 世界坐标 (xyz_in_world / ypr_in_world)
///      参考：spr_vision_try/tasks/auto_aim/solver.hpp (solvePnP + 手眼标定参数)
///
/// ## 输出
///
///   fetch() 输出 ArmorList（std::list<Armor>），每个 Armor 包含：
///     - 2D: center, points（图像坐标）
///     - 3D: xyz_in_world, ypr_in_world, ypd_in_world（世界坐标）
///
/// ## 测试模式
///
///   当未实现上述组件时，fetch() 返回 false，管道进入 LOST 状态。

#include "../../include/pipeline/DataProvider.hpp"
#include "types.hpp"

#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

namespace auto_aim {

// ---- 前置声明（用户在 .cpp 中替换为实际实现） ----
class CameraImpl;
class DetectorImpl;
class PnPSolverImpl;

class AutoAimProvider : public aim::DataProvider<ArmorList> {
public:
  /// @param config_path  配置文件路径（YAML / OpenCV FileStorage 格式）
  explicit AutoAimProvider(const std::string & config_path);
  ~AutoAimProvider() override;

  /// @brief 采集一帧 → 检测 → PnP → 输出
  bool fetch(ArmorList & out_armors) override;

  /// @brief 获取最新原始帧（用于调试显示）
  const cv::Mat & lastFrame() const { return last_frame_; }

private:
  std::unique_ptr<CameraImpl>    camera_;
  std::unique_ptr<DetectorImpl>  detector_;
  std::unique_ptr<PnPSolverImpl> pnp_solver_;

  cv::Mat last_frame_;
  bool initialized_ = false;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PROVIDER_HPP