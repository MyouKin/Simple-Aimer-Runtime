#ifndef AUTO_AIM__PROVIDER_HPP
#define AUTO_AIM__PROVIDER_HPP
/// @file AutoAimProvider.hpp
/// @brief 数据提供层 — 相机采集 + 传统检测 → PnP 3D 解算（待接入）
///
/// 对应 spr_vision_try 整个 main 线程前半段：
///   Camera → Detector（传统灯条检测）→ Solver（PnP）
///
/// 作为 Simple-Aimer-Runtime 的 DataProvider<ArmorList> 实现。
///
/// InputType = ArmorList (std::list<Armor>)
///   每个 Armor 已填入 xyz_in_world / ypr_in_world / ypd_in_world

#include "../../include/pipeline/DataProvider.hpp"
#include "../types.hpp"

#include "detector.hpp"
#include "pnp_solver.hpp"
#include "yolo.hpp"

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>

namespace io {
class Camera;
}  // namespace io

namespace auto_aim {

class AutoAimProvider : public aim::DataProvider<ArmorList> {
public:
  /// @param config_path  主配置文件路径 (standard4.yaml)
  explicit AutoAimProvider(const std::string & config_path);
  ~AutoAimProvider() override;

  /// @brief 采集一帧 → 传统检测 → 返回 ArmorList
  /// @param out_armors  输出装甲板列表
  /// @return true 取图成功（即使未检测到装甲板），false 相机取图失败
  bool fetch(ArmorList & out_armors) override;

  /// @brief 接收 Actuator 反馈，更新 PnP 世界变换
  void acceptFeedback(const aim::SelfState & fb) override;

  /// @brief 获取最近一帧图像（供外部调试用）
  const cv::Mat & lastFrame() const { return last_frame_; }

private:
  cv::Mat last_frame_;
  std::chrono::steady_clock::time_point last_timestamp_;

  std::unique_ptr<io::Camera> camera_;
  std::unique_ptr<Detector>  detector_;
  std::unique_ptr<YOLO>      yolo_;
  std::unique_ptr<PnPSolver> pnp_solver_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PROVIDER_HPP
