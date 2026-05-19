#ifndef AIM_FRAMEWORK_DEBUG_IMGUI_DEBUGGER_HPP
#define AIM_FRAMEWORK_DEBUG_IMGUI_DEBUGGER_HPP

#include <imgui.h>
#include <string>
#include <unordered_map>
#include <vector>

struct GLFWwindow;

namespace aim {

struct CurveAxis {
  char   name[64] = "";
  std::vector<std::string> curves;
  bool   visible = true;
};

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
  std::vector<CurveAxis> axes_;
  std::unordered_map<std::string, bool> curve_visible_;
};

} // namespace aim
#endif
