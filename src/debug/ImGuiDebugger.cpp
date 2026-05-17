#include "../../include/debug/ImGuiDebugger.hpp"
#include "../../include/core/Registry.hpp"
#include "../../include/core/DebugContext.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <iostream>

namespace aim {

ImGuiDebugger::ImGuiDebugger() {}

ImGuiDebugger::~ImGuiDebugger() {
    cleanup();
}

void ImGuiDebugger::init() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return;
    }

#if defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    window_ = glfwCreateWindow(1280, 720, "SimpleAimer Debugger", nullptr, nullptr);
    if (!window_) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return;
    }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1); // Enable vsync

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
}

void ImGuiDebugger::cleanup() {
    if (window_) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        glfwDestroyWindow(window_);
        glfwTerminate();
        window_ = nullptr;
    }
}

void ImGuiDebugger::run() {
    init();
    if (!window_) return;

    while (!glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        renderFrame();

        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window_, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window_);
    }
}

void ImGuiDebugger::renderFrame() {
    renderRegistryPanel();
    renderCurvesPanel();
    render2DTargetPanel();
    renderImagesPanel();
}

void ImGuiDebugger::renderRegistryPanel() {
    ImGui::Begin("Parameters");
    auto& registry = ParameterRegistry::getInstance();
    
    for (const auto& param : registry.getFloatParams()) {
        ImGui::SliderFloat(param.name.c_str(), reinterpret_cast<float*>(param.value), param.min_val, param.max_val);
    }
    
    for (const auto& param : registry.getIntParams()) {
        ImGui::SliderInt(param.name.c_str(), param.value, param.min_val, param.max_val);
    }
    
    ImGui::End();
}

void ImGuiDebugger::renderCurvesPanel() {
    ImGui::Begin("Curves");
    auto& context = DebugContext::getInstance();
    std::lock_guard<std::mutex> lock(context.getMutex());
    
    for (const auto& [name, curve] : context.getCurves()) {
        if (!curve.values.empty()) {
            ImGui::Text("%s: %.4f", name.c_str(), curve.values.back());
            std::vector<float> continuous_data(curve.values.begin(), curve.values.end());
            ImGui::PlotLines(("##" + name).c_str(), continuous_data.data(), continuous_data.size(), 0, nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 100));
        }
    }
    ImGui::End();
}

void ImGuiDebugger::render2DTargetPanel() {
    ImGui::Begin("2D Target View");
    
    ImVec2 p = ImGui::GetCursorScreenPos();
    float width = 400.0f;
    float height = 300.0f;
    ImGui::InvisibleButton("##canvas", ImVec2(width, height));
    
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(p, ImVec2(p.x + width, p.y + height), IM_COL32(50, 50, 50, 255));
    draw_list->AddRect(p, ImVec2(p.x + width, p.y + height), IM_COL32(255, 255, 255, 255));

    // Draw center crosshair
    ImVec2 center(p.x + width / 2.0f, p.y + height / 2.0f);
    draw_list->AddLine(ImVec2(center.x - 10, center.y), ImVec2(center.x + 10, center.y), IM_COL32(100, 100, 100, 255));
    draw_list->AddLine(ImVec2(center.x, center.y - 10), ImVec2(center.x, center.y + 10), IM_COL32(100, 100, 100, 255));

    auto& context = DebugContext::getInstance();
    Vec2 target_point;
    if (context.getTarget2D(target_point)) {
        // Map target_point (-1.0 to 1.0 or whatever coordinates) to canvas
        // Assuming target_point is in image coordinates, e.g. 0-640, 0-480
        // We will just draw a relative point for demonstration
        ImVec2 target_pos(p.x + target_point.x(), p.y + target_point.y());
        draw_list->AddCircleFilled(target_pos, 5.0f, IM_COL32(0, 255, 0, 255));
    }

    ImGui::End();
}

#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif

void ImGuiDebugger::renderImagesPanel() {
    auto images = DebugContext::getInstance().getImages();
    
    for (const auto& [name, mat] : images) {
        if (mat.empty()) continue;

        unsigned int& tex_id = image_textures_[name];
        if (tex_id == 0) {
            glGenTextures(1, &tex_id);
        }

        glBindTexture(GL_TEXTURE_2D, tex_id);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);

        // Upload OpenCV BGR matrix to OpenGL texture
        // cv::Mat mask is typically single channel (grayscale/binary)
        if (mat.channels() == 1) {
            // macOS core profile (GL 3.2+) or standard GL3 doesn't easily support LUMINANCE anymore.
            // A simple trick to avoid GL_TEXTURE_SWIZZLE (not in base GL3 headers sometimes) 
            // is to convert the 1-channel mat to BGR right before uploading.
            cv::Mat cmat;
            cv::cvtColor(mat, cmat, cv::COLOR_GRAY2BGR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, cmat.cols, cmat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, cmat.data);
        } else if (mat.channels() == 3) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
        } else if (mat.channels() == 4) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mat.cols, mat.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, mat.data);
        }

        ImGui::Begin(name.c_str());
        ImVec2 avail = ImGui::GetContentRegionAvail();
        if (avail.x > 0.0f) {
            float aspect = (float)mat.cols / mat.rows;
            float display_w = avail.x;
            float display_h = display_w / aspect;
            ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(tex_id)), ImVec2(display_w, display_h));
        }
        ImGui::End();
    }
}

} // namespace aim
