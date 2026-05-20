/// @file AimerLogger.cpp
/// @brief 独立日志窗口实现 —— 自维护缓冲 + spdlog 控制台/文件输出

#include "../../include/debug/AimerLogger.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <ctime>

namespace aim {

// ---- GLFW 一次性初始化 ----
static void ensureGlfw() {
    static bool ok = []() { glfwInit(); return true; }();
    (void)ok;
}

// ---- 单例 ----
AimerLogger& AimerLogger::instance() {
    static AimerLogger inst;
    return inst;
}

AimerLogger::AimerLogger() {
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::debug);

    auto t = std::time(nullptr);
    char fname[64];
    std::strftime(fname, sizeof(fname), "logs/%Y-%m-%d_%H-%M-%S.log", std::localtime(&t));
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(fname, true);
    file_sink->set_level(spdlog::level::debug);

    spdlog::sinks_init_list sinks = { console_sink, file_sink };
    spdlog_logger_ = std::make_shared<spdlog::logger>("aimer", sinks);
    spdlog_logger_->set_level(spdlog::level::debug);
    spdlog_logger_->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
    spdlog_logger_->flush_on(spdlog::level::info);
    spdlog::set_default_logger(spdlog_logger_);
}

AimerLogger::~AimerLogger() { hideWindow(); }

// ---- 日志接口（线程安全：写入自维护缓冲 + spdlog 输出） ----
void AimerLogger::info(const std::string& msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.push_back({2, msg, std::chrono::system_clock::now()});
        if (buffer_.size() > buffer_max_) buffer_.erase(buffer_.begin());
    }
    spdlog_logger_->info(msg);
}
void AimerLogger::warn(const std::string& msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.push_back({3, msg, std::chrono::system_clock::now()});
        if (buffer_.size() > buffer_max_) buffer_.erase(buffer_.begin());
    }
    spdlog_logger_->warn(msg);
}
void AimerLogger::error(const std::string& msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.push_back({4, msg, std::chrono::system_clock::now()});
        if (buffer_.size() > buffer_max_) buffer_.erase(buffer_.begin());
    }
    spdlog_logger_->error(msg);
}
void AimerLogger::debug(const std::string& msg) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        buffer_.push_back({0, msg, std::chrono::system_clock::now()});
        if (buffer_.size() > buffer_max_) buffer_.erase(buffer_.begin());
    }
    spdlog_logger_->debug(msg);
}

std::shared_ptr<spdlog::logger> AimerLogger::spdlogLogger() { return spdlog_logger_; }

// ---- 窗口 ----
void AimerLogger::showWindow() {
    if (running_) return;
    running_ = true;
    thread_ = std::thread(&AimerLogger::windowThread, this);
}

void AimerLogger::hideWindow() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

bool AimerLogger::isWindowOpen() const { return running_; }

// ---- 日志级别颜色 ----
static ImVec4 levelColor(int lv) {
    switch (lv) {
    case 0: return ImVec4(0.55f, 0.55f, 0.55f, 1.0f); // debug gray
    case 2: return ImVec4(0.50f, 0.90f, 0.50f, 1.0f); // info  green
    case 3: return ImVec4(1.00f, 0.90f, 0.20f, 1.0f); // warn  yellow
    case 4: return ImVec4(1.00f, 0.25f, 0.25f, 1.0f); // error red
    default:return ImVec4(1.00f, 1.00f, 1.00f, 1.0f);
    }
}

void AimerLogger::windowThread() {
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

    window_ = glfwCreateWindow(800, 500, "Aimer Logger", nullptr, nullptr);
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

    bool auto_scroll = true;

    while (running_ && !glfwWindowShouldClose(window_)) {
        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- 日志窗口 UI（填满整个窗口） ----
        ImGui::SetNextWindowPos(ImVec2(0, 0));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
        ImGui::Begin("Logger", nullptr,
                     ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                     ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);

        // 工具栏
        if (ImGui::Button("Clear")) {
            std::lock_guard<std::mutex> lk(mutex_);
            buffer_.clear();
        }
        ImGui::SameLine();
        ImGui::Checkbox("Auto-scroll", &auto_scroll);
        ImGui::SameLine();
        ImGui::TextUnformatted("| select text + Ctrl+C to copy");
        ImGui::Separator();

        // 日志内容区域（可选中、复制）
        ImGui::BeginChild("LogScroll", ImVec2(0, 0), false,
                          ImGuiWindowFlags_HorizontalScrollbar);

        {
            std::lock_guard<std::mutex> lk(mutex_);
            for (auto& e : buffer_) {
                auto tt = std::chrono::system_clock::to_time_t(e.time);
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    e.time.time_since_epoch()) % 1000;
                char ts[16];
                std::strftime(ts, sizeof(ts), "%H:%M:%S", std::localtime(&tt));
                ImGui::TextColored(ImVec4(0.45f, 0.45f, 0.45f, 1.0f),
                                   "%s.%03d", ts, (int)ms.count());
                ImGui::SameLine();

                const char* tag = "???";
                switch (e.level) {
                case 0: tag = "[DBG]"; break;
                case 2: tag = "[INF]"; break;
                case 3: tag = "[WRN]"; break;
                case 4: tag = "[ERR]"; break;
                }
                ImGui::TextColored(levelColor(e.level), "%s", tag);
                ImGui::SameLine();
                ImGui::TextUnformatted(e.text.c_str());
            }
        }

        if (auto_scroll && ImGui::GetScrollY() < ImGui::GetScrollMaxY() - 10.0f)
            auto_scroll = false; // 用户手动滚动了
        if (auto_scroll)
            ImGui::SetScrollHereY(1.0f);

        ImGui::EndChild();
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
