#ifndef AIM_FRAMEWORK_DEBUG_AIMER_IMAGE_HPP
#define AIM_FRAMEWORK_DEBUG_AIMER_IMAGE_HPP

/// @file AimerImage.hpp
/// @brief 独立图像窗口 —— 多实例，每个实例是一个独立的操作系统窗口
///
/// 用法：
///   AimerImage cam("Camera 1");
///   cam.show(mat);           // 更新图像数据（线程安全）
///   cam.openWindow();        // 打开窗口（独立 OS 窗口）
///   cam.closeWindow();

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <opencv2/opencv.hpp>

struct GLFWwindow;
struct ImGuiContext;

namespace aim {

class AimerImage {
public:
    explicit AimerImage(const std::string& name);
    ~AimerImage();

    /// @brief 更新要显示的图像（线程安全，可从 pipeline 线程调用）
    void show(const cv::Mat& image);

    /// @brief 窗口控制
    void openWindow();
    void closeWindow();
    bool isWindowOpen() const;

    const std::string& name() const { return name_; }

private:
    void windowThread();

    std::string name_;
    cv::Mat image_;
    std::mutex mutex_;
    unsigned int texture_id_ = 0;
    bool texture_created_ = false;
    cv::Size last_image_size_{};

    std::thread thread_;
    std::atomic<bool> running_{false};
    GLFWwindow* window_ = nullptr;
    ImGuiContext* imgui_context_ = nullptr;
};

} // namespace aim
#endif
