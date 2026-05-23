/// @file AimerImage.cpp
/// @brief 独立图像窗口实现 —— 支持多实例

#include "../../include/debug/AimerImage.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <mutex>

namespace aim {

// ---- GLFW 一次性初始化 ----
static void ensureGlfw() {
    static bool ok = []() {
        glfwInit();
        return true;
    }();
    (void)ok;
}

static std::mutex& renderMutex() {
    static std::mutex mutex;
    return mutex;
}

#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif

#ifndef GL_BGRA
#define GL_BGRA 0x80E1
#endif

AimerImage::AimerImage(const std::string& name) : name_(name) {}

AimerImage::~AimerImage() { closeWindow(); }

void AimerImage::show(const cv::Mat& image) {
    std::lock_guard<std::mutex> lk(mutex_);
    image.copyTo(image_);
}

void AimerImage::openWindow() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&AimerImage::windowThread, this);
}

void AimerImage::closeWindow() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool AimerImage::isWindowOpen() const { return running_; }

void AimerImage::windowThread() {
    ensureGlfw();
    std::unique_lock<std::mutex> render_lock(renderMutex());

#if __APPLE__
    const char* glsl = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glsl = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    window_ = glfwCreateWindow(960, 720, name_.c_str(), nullptr, nullptr);
    if (!window_) { running_ = false; return; }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    imgui_context_ = ImGui::CreateContext();
    ImGui::SetCurrentContext(imgui_context_);
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.FontGlobalScale = 1.2f;
    ImGui::GetStyle().ScaleAllSizes(1.2f);
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl);
    render_lock.unlock();

    while (running_ && !glfwWindowShouldClose(window_)) {
        render_lock.lock();
        glfwMakeContextCurrent(window_);
        ImGui::SetCurrentContext(imgui_context_);
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- 图像窗口 UI ----
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin(name_.c_str(), nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                     ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

        cv::Mat local;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!image_.empty()) {
                image_.copyTo(local);
            }
        }

        if (!local.empty()) {
            if (last_image_size_ != local.size()) {
                last_image_size_ = local.size();
                glfwSetWindowAspectRatio(window_, std::max(1, local.cols), std::max(1, local.rows));
                glfwSetWindowSize(window_, std::max(1, local.cols), std::max(1, local.rows));
            }
            // 创建/更新 OpenGL 纹理
            if (!texture_created_) {
                glGenTextures(1, &texture_id_);
                texture_created_ = true;
            }

            glBindTexture(GL_TEXTURE_2D, texture_id_);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

            if (local.channels() == 1) {
                cv::Mat rgb;
                cv::cvtColor(local, rgb, cv::COLOR_GRAY2BGR);
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, rgb.cols, rgb.rows,
                             0, GL_BGR, GL_UNSIGNED_BYTE, rgb.data);
            } else if (local.channels() == 3) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, local.cols, local.rows,
                             0, GL_BGR, GL_UNSIGNED_BYTE, local.data);
            } else if (local.channels() == 4) {
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, local.cols, local.rows,
                             0, GL_BGRA, GL_UNSIGNED_BYTE, local.data);
            }

            // 按比例缩放以适配窗口
            ImVec2 avail = ImGui::GetContentRegionAvail();
            float img_w = static_cast<float>(local.cols);
            float img_h = static_cast<float>(local.rows);
            float scale = std::min(avail.x / img_w, avail.y / img_h);
            if (scale > 0.0f && avail.x > 1.0f && avail.y > 1.0f) {
                ImVec2 image_size(img_w * scale, img_h * scale);
                ImVec2 cursor = ImGui::GetCursorPos();
                ImGui::SetCursorPos(ImVec2(
                    cursor.x + std::max(0.0f, (avail.x - image_size.x) * 0.5f),
                    cursor.y + std::max(0.0f, (avail.y - image_size.y) * 0.5f)));
                ImGui::Image((void*)(intptr_t)texture_id_, image_size);
            }
        } else {
            ImGui::TextUnformatted("(no image)");
        }

        ImGui::End();
        ImGui::PopStyleVar();

        // ---- 渲染 ----
        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
        render_lock.unlock();
    }

    // 清理
    render_lock.lock();
    glfwMakeContextCurrent(window_);
    ImGui::SetCurrentContext(imgui_context_);

    if (texture_created_) {
        glDeleteTextures(1, &texture_id_);
        texture_created_ = false;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(imgui_context_);
    imgui_context_ = nullptr;
    glfwDestroyWindow(window_);
    window_ = nullptr;
    running_ = false;
    render_lock.unlock();
}

} // namespace aim
