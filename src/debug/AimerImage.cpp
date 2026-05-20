/// @file AimerImage.cpp
/// @brief 独立图像窗口实现 —— 支持多实例

#include "../../include/debug/AimerImage.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

namespace aim {

// ---- GLFW 一次性初始化 ----
static void ensureGlfw() {
    static bool ok = []() {
        glfwInit();
        return true;
    }();
    (void)ok;
}

#ifndef GL_BGR
#define GL_BGR 0x80E0
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

    window_ = glfwCreateWindow(640, 480, name_.c_str(), nullptr, nullptr);
    if (!window_) { running_ = false; return; }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.FontGlobalScale = 1.2f;
    ImGui::GetStyle().ScaleAllSizes(1.2f);
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl);

    while (running_ && !glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- 图像窗口 UI ----
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin(name_.c_str(), nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        cv::Mat local;
        {
            std::lock_guard<std::mutex> lk(mutex_);
            if (!image_.empty()) {
                image_.copyTo(local);
            }
        }

        if (!local.empty()) {
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
            float avail_w = ImGui::GetContentRegionAvail().x;
            float avail_h = ImGui::GetContentRegionAvail().y;
            float img_w = (float)local.cols;
            float img_h = (float)local.rows;
            float scale = std::min(avail_w / img_w, avail_h / img_h);
            if (scale > 0.0f && scale < 1.0f) {
                img_w *= scale;
                img_h *= scale;
            }
            ImGui::Image((void*)(intptr_t)texture_id_, ImVec2(img_w, img_h));
        } else {
            ImGui::TextUnformatted("(no image)");
        }

        ImGui::End();

        // ---- 渲染 ----
        ImGui::Render();
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window_);
    }

    // 清理
    if (texture_created_) {
        glDeleteTextures(1, &texture_id_);
        texture_created_ = false;
    }
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    window_ = nullptr;
    running_ = false;
}

} // namespace aim
