#include "LaserAimerProvider.hpp"
#include "../../include/core/Registry.hpp"
#include "../../include/core/DebugContext.hpp"
#include <cmath>
#include <iostream>

namespace aim {

LaserAimerProvider::LaserAimerProvider() {
    // ---------- MindVision SDK 初始化 ----------
    CameraSdkInit(1);

    // 枚举设备
    int camera_counts = 1;
    tSdkCameraDevInfo camera_enum_list;
    int status = CameraEnumerateDevice(&camera_enum_list, &camera_counts);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[LaserAimerProvider] CameraEnumerateDevice failed, status = "
                  << status << std::endl;
        h_camera_ = -1;
        return;
    }
    if (camera_counts == 0) {
        std::cerr << "[LaserAimerProvider] No MindVision camera found!" << std::endl;
        h_camera_ = -1;
        return;
    }
    std::cout << "[LaserAimerProvider] Found " << camera_counts << " camera(s)" << std::endl;

    // 初始化相机
    status = CameraInit(&camera_enum_list, -1, -1, &h_camera_);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[LaserAimerProvider] CameraInit failed, status = " << status << std::endl;
        h_camera_ = -1;
        return;
    }

    // 获取相机能力描述
    CameraGetCapability(h_camera_, &t_capability_);

    // 为 RGB 缓冲区预分配内存 (参照 mv_camera_node: 仅 reserve, 待拿到帧头后才 resize)
    rgb_buffer_.reserve(
        t_capability_.sResolutionRange.iHeightMax *
        t_capability_.sResolutionRange.iWidthMax * 3);

    // ---------- 设置相机参数 (严格对齐 mv_camera_node.cpp) ----------
    // 1) 关闭自动曝光，使用手动曝光
    CameraSetAeState(h_camera_, false);

    // 2) 设置曝光时间
    //    参照 mv_camera_node: 先 GetExposureLineTime 获取时间基准
    double exposure_line_time;
    CameraGetExposureLineTime(h_camera_, &exposure_line_time);
    CameraSetExposureTime(h_camera_, exposure_time_);
    std::cout << "[LaserAimerProvider] Exposure time set to " << exposure_time_ << std::endl;

    // 3) 设置模拟增益
    //    参照 mv_camera_node: 先 GetAnalogGain 获取当前值
    int current_gain = 0;
    CameraGetAnalogGain(h_camera_, &current_gain);
    CameraSetAnalogGain(h_camera_, analog_gain_);
    std::cout << "[LaserAimerProvider] Analog gain set to " << analog_gain_
              << " (was " << current_gain << ")" << std::endl;

    // 4) 让相机进入工作模式，开始接收图像数据
    CameraPlay(h_camera_);

    // 5) 设置输出格式为 RGB8（必须在 CameraPlay 之后）
    CameraSetIspOutFormat(h_camera_, CAMERA_MEDIA_TYPE_RGB8);

    std::cout << "[LaserAimerProvider] Camera started (PLAY mode)" << std::endl;

    // ---------- 注册 ImGui 可调参数 ----------
    auto& reg = ParameterRegistry::getInstance();
    reg.registerInt("Exposure Time", &exposure_time_, 1, 100000);
    reg.registerInt("Analog Gain", &analog_gain_, 0, 1023);
    reg.registerInt("H Min", &h_min_, 0, 179);
    reg.registerInt("H Max", &h_max_, 0, 179);
    reg.registerInt("S Min", &s_min_, 0, 255);
    reg.registerInt("S Max", &s_max_, 0, 255);
    reg.registerInt("V Min", &v_min_, 0, 255);
    reg.registerInt("V Max", &v_max_, 0, 255);
    reg.registerInt("Close Kernel", &close_kernel_, 1, 50);
    reg.registerInt("Min Area", &min_area_, 10, 5000);
    reg.registerInt("Max Area", &max_area_, 1000, 20000);
    reg.registerInt("Min Aspect (x10)", &min_aspect_x10_, 10, 100);
    reg.registerInt("Max Aspect (x10)", &max_aspect_x10_, 10, 500);
    reg.registerInt("Max Angle Diff", &max_angle_diff_, 5, 90);
    reg.registerInt("Min Dist Ratio (x10)", &min_dist_ratio_x10_, 5, 100);
    reg.registerInt("Max Dist Ratio (x10)", &max_dist_ratio_x10_, 10, 150);
}

LaserAimerProvider::~LaserAimerProvider() {
    if (h_camera_ >= 0) {
        CameraUnInit(h_camera_);
        std::cout << "[LaserAimerProvider] Camera uninitialized" << std::endl;
    }
}

struct LightStrip {
    cv::Point2f center;
    double length;
    double angle_deg;
    cv::Point2f primary_vec;
    cv::Point2f secondary_vec;
};

bool LaserAimerProvider::fetch(std::optional<aim::FinalTargetState>& out_data) {
    if (h_camera_ < 0) {
        out_data = std::nullopt;
        return false;
    }

    // ---------- 从 MindVision 相机抓取一帧 ----------
    int status = CameraGetImageBuffer(h_camera_, &s_frame_info_, &pby_buffer_, 1000);
    if (status != CAMERA_STATUS_SUCCESS) {
        std::cerr << "[LaserAimerProvider] CameraGetImageBuffer failed, status = "
                  << status << std::endl;
        out_data = std::nullopt;
        return false;
    }

    // 将原始 Bayer 转换为 RGB8
    CameraImageProcess(h_camera_, pby_buffer_, rgb_buffer_.data(), &s_frame_info_);

    // 可选翻转（参照 mv_camera_node 的 flip_image 参数）
    if (flip_image_) {
        CameraFlipFrameBuffer(rgb_buffer_.data(), &s_frame_info_, 3);
    }

    // 释放 SDK 图像缓冲
    CameraReleaseImageBuffer(h_camera_, pby_buffer_);

    // 按实际帧尺寸调整缓冲区大小（参照 mv_camera_node 的做法）
    int width = s_frame_info_.iWidth;
    int height = s_frame_info_.iHeight;
    rgb_buffer_.resize(width * height * 3);

    // 构建 OpenCV Mat（SDK 输出 RGB8，但 OpenCV 及 ImGui 调试器均使用 BGR 顺序）
    cv::Mat frame(height, width, CV_8UC3, rgb_buffer_.data());
    cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);

    // ---------- 以下为原有的图像处理 / 激光点检测逻辑 ----------
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    if (h_min_ <= h_max_) {
        cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
                    cv::Scalar(h_max_, s_max_, v_max_), mask);
    } else {
        cv::Mat mask1, mask2;
        cv::inRange(hsv, cv::Scalar(0, s_min_, v_min_),
                    cv::Scalar(h_max_, s_max_, v_max_), mask1);
        cv::inRange(hsv, cv::Scalar(h_min_, s_min_, v_min_),
                    cv::Scalar(179, s_max_, v_max_), mask2);
        cv::bitwise_or(mask1, mask2, mask);
    }

    int k_size = std::max(1, close_kernel_);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_size, k_size));
    cv::Mat mask_closed;
    cv::morphologyEx(mask, mask_closed, cv::MORPH_CLOSE, kernel);
    DebugContext::getInstance().setImage("Binary Mask", mask_closed);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Mat result_img = frame.clone();
    std::vector<LightStrip> strips;

    for (const auto& cnt : contours) {
        double area = cv::contourArea(cnt);
        if (area < min_area_ || area > max_area_) continue;

        cv::RotatedRect rect = cv::minAreaRect(cnt);
        double width_r = rect.size.width;
        double height_r = rect.size.height;
        if (width_r == 0 || height_r == 0) continue;

        double length = std::max(width_r, height_r);
        double short_side = std::min(width_r, height_r);
        double aspect_ratio = length / short_side;

        if (aspect_ratio < (min_aspect_x10_ / 10.0) ||
            aspect_ratio > (max_aspect_x10_ / 10.0))
            continue;

        cv::Mat data_pts(cnt.size(), 2, CV_64F);
        for (size_t i = 0; i < cnt.size(); i++) {
            data_pts.at<double>(i, 0) = cnt[i].x;
            data_pts.at<double>(i, 1) = cnt[i].y;
        }
        cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

        cv::Point2f center(pca.mean.at<double>(0, 0), pca.mean.at<double>(0, 1));
        cv::Point2f primary(pca.eigenvectors.at<double>(0, 0),
                            pca.eigenvectors.at<double>(0, 1));
        cv::Point2f secondary(pca.eigenvectors.at<double>(1, 0),
                              pca.eigenvectors.at<double>(1, 1));

        double angle = std::atan2(primary.y, primary.x) * 180.0 / M_PI;

        strips.push_back({center, length, angle, primary, secondary});

        cv::Point2f pts[4];
        rect.points(pts);
        for (int i = 0; i < 4; i++) {
            cv::line(result_img, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
        }
    }

    out_data = std::nullopt;
    for (size_t i = 0; i < strips.size(); i++) {
        for (size_t j = i + 1; j < strips.size(); j++) {
            const auto& s1 = strips[i];
            const auto& s2 = strips[j];

            double angle_diff = std::abs(s1.angle_deg - s2.angle_deg);
            if (angle_diff > 90.0) angle_diff = 180.0 - angle_diff;
            if (angle_diff > max_angle_diff_) continue;

            cv::Point2f diff = s1.center - s2.center;
            double center_dist = cv::norm(diff);
            double avg_length = (s1.length + s2.length) / 2.0;
            double dist_ratio = center_dist / avg_length;

            if (dist_ratio < (min_dist_ratio_x10_ / 10.0) ||
                dist_ratio > (max_dist_ratio_x10_ / 10.0))
                continue;

            cv::Point2f avg_secondary = (s1.secondary_vec + s2.secondary_vec) * 0.5f;
            double norm = cv::norm(avg_secondary);
            if (norm > 0) avg_secondary /= norm;

            aim::FinalTargetState target;
            target.has_image_point = true;
            cv::Point2f target_center = (s1.center + s2.center) * 0.5f;
            target.image_point = Vec2(target_center.x, target_center.y);
            target.valid = true;
            target.timestamp = std::chrono::high_resolution_clock::now();
            out_data = target;

            cv::line(result_img, s1.center, s2.center, cv::Scalar(255, 0, 0), 2);
            cv::circle(result_img, target_center, 5, cv::Scalar(0, 255, 255), -1);
            break;
        }
        if (out_data.has_value()) break;
    }

    DebugContext::getInstance().setImage("Detection Result", result_img);
    return true;
}

} // namespace aim