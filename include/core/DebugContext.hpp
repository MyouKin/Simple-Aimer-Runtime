#ifndef AIM_FRAMEWORK_CORE_DEBUG_CONTEXT_HPP
#define AIM_FRAMEWORK_CORE_DEBUG_CONTEXT_HPP

#include "types.hpp"
#include <vector>
#include <string>
#include <mutex>
#include <deque>
#include <unordered_map>
#include <opencv2/opencv.hpp>

namespace aim {

struct CurveData {
    std::string name;
    std::deque<float> values;
    size_t max_size = 1000;
};

class DebugContext {
public:
    static DebugContext& getInstance() {
        static DebugContext instance;
        return instance;
    }

    // 画 2D 准星/目标点 (主要给图像上的追踪用)
    void setTarget2D(const Vec2& point, bool valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_2d_ = point;
        target_2d_valid_ = valid;
    }

    bool getTarget2D(Vec2& out_point) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!target_2d_valid_) return false;
        out_point = target_2d_;
        return true;
    }

    void setPoint(const std::string& name, const cv::Point2f& point, bool valid) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (valid) {
            points_[name] = point;
        } else {
            points_.erase(name);
        }
    }

    bool getPoint(const std::string& name, cv::Point2f& out_point) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = points_.find(name);
        if (it == points_.end()) return false;
        out_point = it->second;
        return true;
    }

    void setText(const std::string& name, const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        texts_[name] = text;
    }

    std::string getText(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = texts_.find(name);
        if (it == texts_.end()) return {};
        return it->second;
    }

    // 记录图表数据 (如 yaw/pitch 变化)
    void pushCurveData(const std::string& name, float value) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto& curve = curves_[name];
        curve.name = name;
        curve.values.push_back(value);
        if (curve.values.size() > curve.max_size) {
            curve.values.pop_front();
        }
    }

    const std::unordered_map<std::string, CurveData>& getCurves() {
        // 返回时注意线程安全，通常UI读取时需要拷贝或者在锁内读取
        return curves_;
    }

    // 设置/更新要显示的图片
    void setImage(const std::string& name, const cv::Mat& image) {
        std::lock_guard<std::mutex> lock(mutex_);
        image.copyTo(images_[name]);
    }

    // 获取所有待显示的图片
    std::unordered_map<std::string, cv::Mat> getImages() {
        std::lock_guard<std::mutex> lock(mutex_);
        return images_; // 返回拷贝以避免渲染时一直占用锁
    }
    
    std::mutex& getMutex() { return mutex_; }

private:
    DebugContext() = default;
    ~DebugContext() = default;

    std::mutex mutex_;
    Vec2 target_2d_ = Vec2::Zero();
    bool target_2d_valid_ = false;

    std::unordered_map<std::string, CurveData> curves_;
    std::unordered_map<std::string, cv::Mat> images_;
    std::unordered_map<std::string, cv::Point2f> points_;
    std::unordered_map<std::string, std::string> texts_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_CORE_DEBUG_CONTEXT_HPP
