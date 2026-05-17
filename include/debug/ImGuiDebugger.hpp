#ifndef AIM_FRAMEWORK_DEBUG_IMGUI_DEBUGGER_HPP
#define AIM_FRAMEWORK_DEBUG_IMGUI_DEBUGGER_HPP

#include <string>
#include <unordered_map>

// Forward declaration of GLFWwindow to avoid including GLFW here
struct GLFWwindow;

namespace aim {

class ImGuiDebugger {
public:
    ImGuiDebugger();
    ~ImGuiDebugger();

    void run();

private:
    void init();
    void cleanup();
    void renderFrame();
    void renderRegistryPanel();
    void renderCurvesPanel();
    void render2DTargetPanel();
    void renderImagesPanel();

    GLFWwindow* window_ = nullptr;
    std::unordered_map<std::string, unsigned int> image_textures_;
};

} // namespace aim

#endif // AIM_FRAMEWORK_DEBUG_IMGUI_DEBUGGER_HPP
