/// @file AimerCurve.cpp
/// @brief 独立曲线窗口实现 —— 多坐标轴，用户自由分配曲线

#include "../../include/debug/AimerCurve.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cfloat>
#include <cstdio>

namespace aim {

// ---- GLFW 一次性初始化 ----
static void ensureGlfw() {
    static bool ok = []() {
        glfwInit();
        return true;
    }();
    (void)ok;
}

// ---- 16 色调色板 ----
static ImU32 palette(int i) {
    static const ImU32 p[] = {
        IM_COL32(255,80,80,255),  IM_COL32(80,200,80,255),
        IM_COL32(80,120,255,255), IM_COL32(255,200,50,255),
        IM_COL32(200,80,255,255), IM_COL32(50,200,200,255),
        IM_COL32(255,255,80,255), IM_COL32(255,150,200,255),
        IM_COL32(150,255,100,255),IM_COL32(100,180,255,255),
        IM_COL32(255,100,100,255),IM_COL32(120,255,120,255),
        IM_COL32(120,120,255,255),IM_COL32(255,220,60,255),
        IM_COL32(220,100,255,255),IM_COL32(100,220,220,255),
    };
    return p[i % 16];
}

AimerCurve::AimerCurve(const std::string& name) : name_(name) {}

AimerCurve::~AimerCurve() { closeWindow(); }

void AimerCurve::push(const std::string& curve_name, float value) {
    std::lock_guard<std::mutex> lk(mutex_);
    auto& dq = curves_[curve_name];
    dq.push_back(value);
    if (dq.size() > max_points_) dq.pop_front();
}

void AimerCurve::openWindow() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&AimerCurve::windowThread, this);
}

void AimerCurve::closeWindow() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool AimerCurve::isWindowOpen() const { return running_; }

void AimerCurve::windowThread() {
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

    window_ = glfwCreateWindow(900, 600, name_.c_str(), nullptr, nullptr);
    if (!window_) { running_ = false; return; }

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.IniFilename = nullptr;
    io.FontGlobalScale = 1.3f;
    ImGui::GetStyle().ScaleAllSizes(1.3f);
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window_, true);
    ImGui_ImplOpenGL3_Init(glsl);

    while (running_ && !glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- 曲线窗口 UI ----
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin(name_.c_str(), nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        {
            std::lock_guard<std::mutex> lk(mutex_);

            if (curves_.empty()) {
                ImGui::TextUnformatted("(no curve data — push values with AimerCurve::push())");
            }

            // ---- 工具栏 ----
            if (ImGui::Button("+ Axis")) {
                CurveAxis ax;
                snprintf(ax.name, sizeof(ax.name), "axis %zu", axes_.size() + 1);
                axes_.push_back(ax);
            }
            ImGui::SameLine();
            if (ImGui::Button("Show All"))
                for (auto& kv : curve_visible_) kv.second = true;
            ImGui::SameLine();
            if (ImGui::Button("Hide All"))
                for (auto& kv : curve_visible_) kv.second = false;
            ImGui::Separator();

            // ---- 每个坐标轴 ----
            for (int ai = 0; ai < (int)axes_.size(); ++ai) {
                auto& ax = axes_[ai];
                ImGui::PushID(ai);

                // 可见性 + 名称
                ImGui::Checkbox("##v", &ax.visible);
                ImGui::SameLine();
                char nid[32];
                snprintf(nid, sizeof(nid), "##n%d", ai);
                ImGui::SetNextItemWidth(150);
                ImGui::InputText(nid, ax.name, sizeof(ax.name));
                ImGui::SameLine();
                if (ImGui::SmallButton("X")) {
                    axes_.erase(axes_.begin() + ai);
                    ImGui::PopID();
                    --ai;
                    continue;
                }

                // 添加曲线 Combo
                if (ImGui::BeginCombo("##add", "+ curve")) {
                    for (auto& [cn, _] : curves_) {
                        bool added = std::find(ax.curves.begin(), ax.curves.end(), cn) !=
                                     ax.curves.end();
                        if (!added && ImGui::Selectable(cn.c_str())) {
                            ax.curves.push_back(cn);
                            if (curve_visible_.find(cn) == curve_visible_.end())
                                curve_visible_[cn] = true;
                        }
                    }
                    ImGui::EndCombo();
                }

                // 曲线 Checkbox（倒序显示）
                if (!ax.curves.empty()) {
                    int ci = 0;
                    for (int ci2 = (int)ax.curves.size() - 1; ci2 >= 0; --ci2) {
                        auto& cn = ax.curves[ci2];
                        ImGui::SameLine(0, 4);
                        ImGui::PushStyleColor(ImGuiCol_CheckMark, palette(ci));
                        bool& vis = curve_visible_[cn];
                        ImGui::Checkbox(cn.c_str(), &vis);
                        ImGui::PopStyleColor();
                        // 双击移除
                        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0))
                            ax.curves.erase(ax.curves.begin() + ci2);
                        ++ci;
                    }
                }

                // ---- 绘图 ----
                if (ax.visible && !ax.curves.empty()) {
                    float ymin = FLT_MAX, ymax = -FLT_MAX;
                    size_t max_sz = 0;
                    for (auto& cn : ax.curves) {
                        if (!curve_visible_[cn]) continue;
                        auto it = curves_.find(cn);
                        if (it == curves_.end() || it->second.empty()) continue;
                        max_sz = std::max(max_sz, it->second.size());
                        for (auto v : it->second) {
                            ymin = std::min(ymin, v);
                            ymax = std::max(ymax, v);
                        }
                    }
                    if (max_sz < 2) { ImGui::PopID(); continue; }
                    if (ymax <= ymin) { ymax = ymin + 1.0f; ymin -= 0.5f; }

                    ImVec2 sz(ImGui::GetContentRegionAvail().x, 140);
                    ImGui::InvisibleButton("##cv", sz);
                    ImDrawList* dl = ImGui::GetWindowDrawList();
                    ImVec2 p0 = ImGui::GetItemRectMin(), p1 = ImGui::GetItemRectMax();
                    dl->AddRectFilled(p0, p1, IM_COL32(22, 22, 22, 255));
                    dl->AddRect(p0, p1, IM_COL32(70, 70, 70, 255));

                    // Y 轴标签
                    float H = p1.y - p0.y;
                    auto v2y = [&](float v) {
                        return p1.y - (v - ymin) / (ymax - ymin) * H;
                    };
                    char lb[32];
                    snprintf(lb, sizeof(lb), "%.2f", ymax);
                    dl->AddText(ImVec2(p0.x + 3, p0.y), IM_COL32(200, 200, 200, 255), lb);
                    snprintf(lb, sizeof(lb), "%.2f", ymin);
                    dl->AddText(ImVec2(p0.x + 3, p1.y - 16), IM_COL32(200, 200, 200, 255), lb);

                    // 画曲线
                    int ci = 0;
                    for (auto& cn : ax.curves) {
                        if (!curve_visible_[cn]) {
                            ++ci;
                            continue;
                        }
                        auto it = curves_.find(cn);
                        if (it == curves_.end() || it->second.size() < 2) {
                            ++ci;
                            continue;
                        }
                        auto& vv = it->second;
                        ImU32 col = palette(ci);
                        float xw = (p1.x - p0.x) / std::max(size_t(1), max_sz - 1);
                        auto beg = vv.end() - (std::ptrdiff_t)max_sz;
                        for (size_t i = 1; i < max_sz; ++i) {
                            dl->AddLine(
                                ImVec2(p0.x + (i - 1) * xw, v2y(*(beg + i - 1))),
                                ImVec2(p0.x + i * xw, v2y(*(beg + i))),
                                col, 1.5f);
                        }
                        ++ci;
                    }
                }

                ImGui::PopID();
                ImGui::Separator();
            }
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
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window_);
    window_ = nullptr;
    running_ = false;
}

} // namespace aim
