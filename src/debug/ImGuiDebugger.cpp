#include "../../include/debug/ImGuiDebugger.hpp"
#include "../../include/core/Registry.hpp"
#include "../../include/core/DebugContext.hpp"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cfloat>

namespace aim {

// ---- 16 色调色板 ----
static ImU32 curve_color(int i) {
  static const ImU32 pal[] = {
    IM_COL32(255,80,80,255),  IM_COL32(80,200,80,255),
    IM_COL32(80,120,255,255), IM_COL32(255,200,50,255),
    IM_COL32(200,80,255,255), IM_COL32(50,200,200,255),
    IM_COL32(255,255,80,255), IM_COL32(255,150,200,255),
    IM_COL32(150,255,100,255),IM_COL32(100,180,255,255),
    IM_COL32(255,100,100,255),IM_COL32(120,255,120,255),
    IM_COL32(120,120,255,255),IM_COL32(255,220,60,255),
    IM_COL32(220,100,255,255),IM_COL32(100,220,220,255),
  };
  return pal[i % 16];
}

ImGuiDebugger::ImGuiDebugger() {}
ImGuiDebugger::~ImGuiDebugger() { cleanup(); }

void ImGuiDebugger::init() {
  if (!glfwInit()) return;
#if __APPLE__
  const char* glsl = "#version 150";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,2);
  glfwWindowHint(GLFW_OPENGL_PROFILE,GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT,GL_TRUE);
#else
  const char* glsl = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR,3); glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR,0);
#endif
  window_ = glfwCreateWindow(1280,720,"SimpleAimer Debugger",nullptr,nullptr);
  if (!window_) { glfwTerminate(); return; }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.FontGlobalScale = 2.0f;
  ImGui::GetStyle().ScaleAllSizes(2.0f);
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window_,true);
  ImGui_ImplOpenGL3_Init(glsl);
}

void ImGuiDebugger::cleanup() {
  if (!window_) return;
  ImGui_ImplOpenGL3_Shutdown(); ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext(); glfwDestroyWindow(window_); glfwTerminate();
  window_ = nullptr;
}

void ImGuiDebugger::run() {
  init(); if (!window_) return;
  while (!glfwWindowShouldClose(window_)) {
    glfwPollEvents();
    ImGui_ImplOpenGL3_NewFrame(); ImGui_ImplGlfw_NewFrame(); ImGui::NewFrame();
    renderFrame();
    ImGui::Render();
    int w,h; glfwGetFramebufferSize(window_,&w,&h);
    glViewport(0,0,w,h); glClearColor(0.1f,0.1f,0.1f,1); glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window_);
  }
}

void ImGuiDebugger::renderFrame() {
  renderRegistryPanel();
  renderImagesPanel();
  renderCurvesPanel();
  render2DTargetPanel();
}

void ImGuiDebugger::renderRegistryPanel() {
  ImGui::Begin("Parameters");
  auto& r = ParameterRegistry::getInstance();
  for (auto& p : r.getFloatParams())
    ImGui::SliderFloat(p.name.c_str(),reinterpret_cast<float*>(p.value),p.min_val,p.max_val);
  for (auto& p : r.getIntParams())
    ImGui::SliderInt(p.name.c_str(),p.value,p.min_val,p.max_val);
  ImGui::End();
}

void ImGuiDebugger::render2DTargetPanel() {
  ImGui::Begin("2D Target View");
  ImVec2 p=ImGui::GetCursorScreenPos(); float w=400,h=300;
  ImGui::InvisibleButton("##cv2d",ImVec2(w,h));
  ImDrawList* dl=ImGui::GetWindowDrawList();
  dl->AddRectFilled(p,ImVec2(p.x+w,p.y+h),IM_COL32(50,50,50,255));
  dl->AddRect(p,ImVec2(p.x+w,p.y+h),IM_COL32(255,255,255,255));
  ImVec2 c(p.x+w/2,p.y+h/2);
  dl->AddLine(ImVec2(c.x-10,c.y),ImVec2(c.x+10,c.y),IM_COL32(100,100,100,255));
  dl->AddLine(ImVec2(c.x,c.y-10),ImVec2(c.x,c.y+10),IM_COL32(100,100,100,255));
  Vec2 tp; if (DebugContext::getInstance().getTarget2D(tp))
    dl->AddCircleFilled(ImVec2(p.x+tp.x(),p.y+tp.y()),5,IM_COL32(0,255,0,255));
  ImGui::End();
}

#ifndef GL_BGR
#define GL_BGR 0x80E0
#endif

void ImGuiDebugger::renderImagesPanel() {
  for (auto& [name,mat] : DebugContext::getInstance().getImages()) {
    if (mat.empty()) continue;
    unsigned int& tid=image_textures_[name];
    if (!tid) glGenTextures(1,&tid);
    glBindTexture(GL_TEXTURE_2D,tid);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glPixelStorei(GL_UNPACK_ROW_LENGTH,0);
    if (mat.channels()==1){ cv::Mat c; cv::cvtColor(mat,c,cv::COLOR_GRAY2BGR); glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,c.cols,c.rows,0,GL_BGR,GL_UNSIGNED_BYTE,c.data); }
    else if (mat.channels()==3) glTexImage2D(GL_TEXTURE_2D,0,GL_RGB,mat.cols,mat.rows,0,GL_BGR,GL_UNSIGNED_BYTE,mat.data);
    else if (mat.channels()==4) glTexImage2D(GL_TEXTURE_2D,0,GL_RGBA,mat.cols,mat.rows,0,GL_BGRA,GL_UNSIGNED_BYTE,mat.data);
    ImGui::Begin(name.c_str());
    ImGui::Image((void*)(intptr_t)tid, ImVec2(mat.cols/2.f,mat.rows/2.f));
    ImGui::End();
  }
}

void ImGuiDebugger::renderCurvesPanel() {
  ImGui::Begin("Curves");

  auto& ctx = DebugContext::getInstance();
  std::lock_guard<std::mutex> lock(ctx.getMutex());
  auto& all = ctx.getCurves();

  if (all.empty()) { ImGui::TextUnformatted("(no curve data)"); ImGui::End(); return; }

  // ---- 全局工具栏 ----
  if (ImGui::Button("+ Axis")) {
    CurveAxis ax;
    snprintf(ax.name, sizeof(ax.name), "axis %zu", axes_.size() + 1);
    axes_.push_back(ax);
  }
  ImGui::SameLine();
  if (ImGui::Button("Show All"))  for (auto& [k,v] : curve_visible_) v = true;
  ImGui::SameLine();
  if (ImGui::Button("Hide All"))  for (auto& [k,v] : curve_visible_) v = false;
  ImGui::Separator();

  for (int ai = 0; ai < (int)axes_.size(); ++ai) {
    auto& ax = axes_[ai];
    ImGui::PushID(ai);

    // 可见性 + 轴名称
    ImGui::Checkbox("##v", &ax.visible); ImGui::SameLine();
    char nid[32]; snprintf(nid,sizeof(nid),"##n%d",ai);
    ImGui::SetNextItemWidth(150);
    ImGui::InputText(nid, ax.name, sizeof(ax.name));
    ImGui::SameLine();
    if (ImGui::SmallButton("X")) { axes_.erase(axes_.begin() + ai); ImGui::PopID(); --ai; continue; }

    // 添加曲线下拉
    if (ImGui::BeginCombo("##add", "+ curve")) {
      for (auto& [cn, _] : all) {
        bool added = std::find(ax.curves.begin(), ax.curves.end(), cn) != ax.curves.end();
        if (!added && ImGui::Selectable(cn.c_str())) {
          ax.curves.push_back(cn);
          if (curve_visible_.find(cn) == curve_visible_.end())
            curve_visible_[cn] = true;
        }
      }
      ImGui::EndCombo();
    }

    // 曲线 checkbox
    if (!ax.curves.empty()) {
      int ci = 0;
      for (int ci2 = (int)ax.curves.size() - 1; ci2 >= 0; --ci2) {
        auto& cn = ax.curves[ci2];
        ImGui::SameLine(0, 4);
        ImGui::PushStyleColor(ImGuiCol_CheckMark, curve_color(ci));
        bool& vis = curve_visible_[cn];
        ImGui::Checkbox(cn.c_str(), &vis);
        ImGui::PopStyleColor();
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0))
          ax.curves.erase(ax.curves.begin() + ci2);
        ++ci;
      }
    }

    // 画图
    if (ax.visible && !ax.curves.empty()) {
      // 计算共享 Y 范围
      float ymin = FLT_MAX, ymax = -FLT_MAX;
      size_t max_sz = 0;
      for (auto& cn : ax.curves) {
        if (!curve_visible_[cn]) continue;
        auto it = all.find(cn);
        if (it == all.end() || it->second.values.empty()) continue;
        max_sz = std::max(max_sz, it->second.values.size());
        for (auto v : it->second.values) { ymin = std::min(ymin, v); ymax = std::max(ymax, v); }
      }
      if (max_sz < 2) { ImGui::PopID(); continue; }
      if (ymax <= ymin) { ymax = ymin + 1.0f; ymin -= 0.5f; }

      ImVec2 sz(ImGui::GetContentRegionAvail().x, 120);
      ImGui::InvisibleButton("##cv", sz);
      ImDrawList* dl = ImGui::GetWindowDrawList();
      ImVec2 p0 = ImGui::GetItemRectMin(), p1 = ImGui::GetItemRectMax();
      dl->AddRectFilled(p0, p1, IM_COL32(22, 22, 22, 255));
      dl->AddRect(p0, p1, IM_COL32(70, 70, 70, 255));

      // Y 轴标签
      float H = p1.y - p0.y;
      auto v2y = [&](float v) { return p1.y - (v - ymin) / (ymax - ymin) * H; };
      char lb[32];
      snprintf(lb, sizeof(lb), "%.2f", ymax); dl->AddText(ImVec2(p0.x + 3, p0.y), IM_COL32(200, 200, 200, 255), lb);
      snprintf(lb, sizeof(lb), "%.2f", ymin); dl->AddText(ImVec2(p0.x + 3, p1.y - 16), IM_COL32(200, 200, 200, 255), lb);

      // 画线
      int ci = 0;
      for (auto& cn : ax.curves) {
        if (!curve_visible_[cn]) { ++ci; continue; }
        auto it = all.find(cn);
        if (it == all.end() || it->second.values.size() < 2) { ++ci; continue; }
        auto& vv = it->second.values;
        ImU32 col = curve_color(ci);
        float xw = (p1.x - p0.x) / std::max(size_t(1), max_sz - 1);
        auto beg = vv.end() - max_sz;
        for (size_t i = 1; i < max_sz; ++i) {
          dl->AddLine(ImVec2(p0.x + (i - 1) * xw, v2y(*(beg + i - 1))),
                      ImVec2(p0.x + i * xw,       v2y(*(beg + i))), col, 1.5f);
        }
        ++ci;
      }
    }
    ImGui::PopID();
    ImGui::Separator();
  }
  ImGui::End();
}

} // namespace aim
