#ifndef AIM_APP_LASER_AIMER_PROVIDER_HPP
#define AIM_APP_LASER_AIMER_PROVIDER_HPP

#include "../../include/pipeline/DataProvider.hpp"
#include "../../include/core/types.hpp"
#include <opencv2/opencv.hpp>
#include <optional>

namespace aim {

class LaserAimerProvider : public DataProvider<std::optional<TargetState>> {
public:
    LaserAimerProvider(int camera_index = 0);
    ~LaserAimerProvider() override;

    bool fetch(std::optional<TargetState>& out_data) override;

private:
    cv::VideoCapture cap_;

    // ImGui 可调参数
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