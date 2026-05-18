/// @file AutoAimProvider.cpp
/// @brief DataProvider 实现（存根版本）
///
/// ## 集成步骤
///
/// 将下面的三个 Impl 类替换为实际的相机/检测/PnP实现：
///
/// @code
/// // === CameraImpl: 替换为你的相机驱动 ===
/// // 参考 spr_vision_try/io/camera.hpp 或直接使用 OpenCV VideoCapture
/// class CameraImpl {
/// public:
///   explicit CameraImpl(const std::string & config_path) {
///     // 1. 从 config 读取相机参数（曝光/增益/分辨率）
///     // 2. 初始化相机 SDK（如 MindVision CameraSdkInit）
///     // 3. 启动采集流
///   }
///   bool get(cv::Mat & frame) {
///     // 从相机读取一帧，写入 frame（BGR 格式）
///     return true;
///   }
/// };
///
/// // === DetectorImpl: 替换为你的检测器 ===
/// // 参考 spr_vision_try/tasks/auto_aim/yolo.hpp
/// class DetectorImpl {
/// public:
///   DetectorImpl(const std::string & config_path, bool debug) {
///     // 1. 加载 ONNX 模型（模型文件放在 models/ 目录）
///     // 2. 初始化推理引擎（OpenCV DNN / ONNX Runtime / TensorRT）
///   }
///   bool detect(const cv::Mat & frame, ArmorList & armors,
///               std::chrono::steady_clock::time_point t) {
///     // 1. 预处理图像
///     // 2. 推理 → 获取 class_id, confidence, bbox, keypoints
///     // 3. 对每个检测结果：armors.push_back(Armor(class_id, conf, bbox, keypoints))
///     return !armors.empty();
///   }
/// };
///
/// // === PnPSolverImpl: 替换为你的 PnP 解算器 ===
/// // 参考 spr_vision_try/tasks/auto_aim/solver.hpp
/// class PnPSolverImpl {
/// public:
///   explicit PnPSolverImpl(const std::string & config_path) {
///     // 1. 加载相机内参 (camera_matrix, distort_coeffs)
///     // 2. 加载手眼标定参数 (R_camera2gimbal, t_camera2gimbal)
///     // 3. 加载云台→IMU 标定 (R_gimbal2imubody)
///   }
///   void solveAll(ArmorList & armors) {
///     for (auto & armor : armors) {
///       // 1. solvePnP(armor_points_3d, armor.points, camera_matrix, distort_coeffs)
///       // 2. 坐标变换：camera → gimbal → world
///       // 3. 填入 armor.xyz_in_world / armor.ypr_in_world / armor.ypd_in_world
///     }
///   }
///   void setGimbal2World(const Eigen::Quaterniond & q) {
///     // 从 IMU 四元数更新 R_gimbal2world
///   }
/// };
/// @endcode
///
/// ## 模型文件
///
/// 将 YOLO 模型权重（.onnx）复制到 applications/auto_aim/models/ 目录，
/// 在 config 中通过 model_path 字段指定路径。

#include "AutoAimProvider.hpp"

namespace auto_aim {

// ============================================================================
// 存根实现（编译通过，运行时返回空数据）
// 用户需替换为实际的相机/检测/PnP实现
// ============================================================================

class CameraImpl {
public:
  explicit CameraImpl(const std::string & /*config_path*/) {}
  bool get(cv::Mat & /*frame*/) { return false; }
};

class DetectorImpl {
public:
  DetectorImpl(const std::string & /*config_path*/, bool /*debug*/) {}
  bool detect(const cv::Mat & /*frame*/, ArmorList & /*armors*/,
              std::chrono::steady_clock::time_point /*t*/) { return false; }
};

class PnPSolverImpl {
public:
  explicit PnPSolverImpl(const std::string & /*config_path*/) {}
  void solveAll(ArmorList & /*armors*/) {}
  void setGimbal2World(const Eigen::Quaterniond & /*q*/) {}
};

// ============================================================================
// AutoAimProvider
// ============================================================================

AutoAimProvider::AutoAimProvider(const std::string & config_path) {
  camera_    = std::make_unique<CameraImpl>(config_path);
  detector_  = std::make_unique<DetectorImpl>(config_path, false);
  pnp_solver_ = std::make_unique<PnPSolverImpl>(config_path);
  initialized_ = false;  // 存根模式：无真实硬件
}

AutoAimProvider::~AutoAimProvider() = default;

bool AutoAimProvider::fetch(ArmorList & out_armors) {
  auto t = std::chrono::steady_clock::now();
  if (!camera_->get(last_frame_)) return false;

  out_armors.clear();
  detector_->detect(last_frame_, out_armors, t);
  pnp_solver_->solveAll(out_armors);
  return true;
}

}  // namespace auto_aim