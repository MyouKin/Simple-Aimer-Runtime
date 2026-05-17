#ifndef AIM_APP_LASER_AIMER_PROVIDER_HPP
#define AIM_APP_LASER_AIMER_PROVIDER_HPP

#include "../../include/pipeline/DataProvider.hpp"
#include "../../include/core/types.hpp"
#include "../../include/mvsdk/include/CameraApi.h"
#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>

namespace aim {

class LaserAimerProvider : public DataProvider<std::optional<TargetState>> {
public:
    LaserAimerProvider();
    ~LaserAimerProvider() override;

    bool fetch(std::optional<TargetState>& out_data) override;

private:
    // MindVision 相机句柄与缓冲区
    CameraHandle h_camera_ = -1;
    tSdkCameraCapbility t_capability_;     // 设备能力描述
    tSdkFrameHead s_frame_info_;            // 图像帧头信息
    BYTE* pby_buffer_ = nullptr;            // 原始图像缓冲区
    std::vector<BYTE> rgb_buffer_;          // RGB 处理后的缓冲区

    // 相机参数
    int exposure_time_ = 4000;
    int analog_gain_ = 128;
    bool flip_image_ = false;

    // ImGui 可调参数（用于图像处理 / 激光点检测）
    int h_min_ = 0, h_max_ = 179;
    int s_min_ = 0, s_max_ = 255;
    int v_min_ = 200, v_max_ = 255;
    int close_kernel_ = 10;
    int min_area_ = 50, max_area_ = 10000;
    int min_aspect_x10_ = 15, max_aspect_x10_ = 100;
    int max_angle_diff_ = 15;
    int min_dist_ratio_x10_ = 10, max_dist_ratio_x10_ = 60;
};

} // namespace aim

#endif // AIM_APP_LASER_AIMER_PROVIDER_HPP