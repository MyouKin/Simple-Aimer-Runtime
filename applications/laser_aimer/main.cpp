#include "../../include/core/DebugContext.hpp"
#include "../../include/runtime/Runtime.hpp"
#include "LaserAimerActuator.hpp"
#include "LaserAimerProvider.hpp"
#include "LaserAimerSelector.hpp"
#include "LaserAimerSolver.hpp"
#include "LaserAimerSystem.hpp"
#include "config.hpp"

#include <algorithm>
#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <thread>
#include <vector>

namespace {
std::atomic<bool> g_stop{false};

void signalHandler(int) {
  g_stop.store(true);
}

std::string defaultConfigPath(char ** argv) {
  namespace fs = std::filesystem;
  const std::vector<fs::path> candidates = {
    "applications/laser_aimer/configs/laser_aimer.yaml",
    "configs/laser_aimer.yaml",
    fs::path(argv[0]).parent_path() / "../../configs/laser_aimer.yaml",
  };

  for (const auto & candidate : candidates) {
    std::error_code ec;
    if (fs::exists(candidate, ec)) {
      return candidate.lexically_normal().string();
    }
  }

  return candidates.front().string();
}

struct FixedTargetTrackbars {
  bool created = false;
  int roi_padding_x1000 = 80;
  int target_team = 1;
  int red_h_min = 170;
  int red_h_max = 10;
  int red_s_min = 0;
  int red_s_max = 255;
  int red_v_min = 200;
  int red_v_max = 255;
  int blue_h_min = 100;
  int blue_h_max = 140;
  int blue_s_min = 0;
  int blue_s_max = 255;
  int blue_v_min = 200;
  int blue_v_max = 255;
  int open_kernel = 3;
  int close_kernel = 10;
  int min_area = 50;
  int max_area = 3000;
  int min_aspect_x100 = 150;
  int max_aspect_x100 = 1000;

  void create(const laser_aimer::FixedTargetConfig & cfg) {
    if (created) return;
    roi_padding_x1000 = static_cast<int>(cfg.roi_padding_ratio * 1000.0);
    target_team = cfg.target_team;
    red_h_min = cfg.red_hsv.h_min;
    red_h_max = cfg.red_hsv.h_max;
    red_s_min = cfg.red_hsv.s_min;
    red_s_max = cfg.red_hsv.s_max;
    red_v_min = cfg.red_hsv.v_min;
    red_v_max = cfg.red_hsv.v_max;
    blue_h_min = cfg.blue_hsv.h_min;
    blue_h_max = cfg.blue_hsv.h_max;
    blue_s_min = cfg.blue_hsv.s_min;
    blue_s_max = cfg.blue_hsv.s_max;
    blue_v_min = cfg.blue_hsv.v_min;
    blue_v_max = cfg.blue_hsv.v_max;
    open_kernel = cfg.open_kernel;
    close_kernel = cfg.close_kernel;
    min_area = static_cast<int>(cfg.min_area);
    max_area = static_cast<int>(cfg.max_area);
    min_aspect_x100 = static_cast<int>(cfg.min_aspect * 100.0);
    max_aspect_x100 = static_cast<int>(cfg.max_aspect * 100.0);

    const std::string win = "FixedTarget Params";
    cv::namedWindow(win, cv::WINDOW_NORMAL);
    cv::createTrackbar("roi_pad x1000", win, &roi_padding_x1000, 1000);
    cv::createTrackbar("team 0red 1blue", win, &target_team, 1);
    cv::createTrackbar("red_h_min", win, &red_h_min, 179);
    cv::createTrackbar("red_h_max", win, &red_h_max, 179);
    cv::createTrackbar("red_s_min", win, &red_s_min, 255);
    cv::createTrackbar("red_s_max", win, &red_s_max, 255);
    cv::createTrackbar("red_v_min", win, &red_v_min, 255);
    cv::createTrackbar("red_v_max", win, &red_v_max, 255);
    cv::createTrackbar("blue_h_min", win, &blue_h_min, 179);
    cv::createTrackbar("blue_h_max", win, &blue_h_max, 179);
    cv::createTrackbar("blue_s_min", win, &blue_s_min, 255);
    cv::createTrackbar("blue_s_max", win, &blue_s_max, 255);
    cv::createTrackbar("blue_v_min", win, &blue_v_min, 255);
    cv::createTrackbar("blue_v_max", win, &blue_v_max, 255);
    cv::createTrackbar("open_kernel", win, &open_kernel, 80);
    cv::createTrackbar("close_kernel", win, &close_kernel, 80);
    cv::createTrackbar("min_area", win, &min_area, 20000);
    cv::createTrackbar("max_area", win, &max_area, 50000);
    cv::createTrackbar("min_aspect x100", win, &min_aspect_x100, 3000);
    cv::createTrackbar("max_aspect x100", win, &max_aspect_x100, 3000);
    created = true;
  }

  laser_aimer::FixedTargetConfig read() const {
    laser_aimer::FixedTargetConfig cfg;
    cfg.roi_padding_ratio = roi_padding_x1000 / 1000.0;
    cfg.target_team = target_team == 0 ? 0 : 1;
    cfg.red_hsv = {red_h_min, red_h_max, red_s_min, red_s_max, red_v_min, red_v_max};
    cfg.blue_hsv = {blue_h_min, blue_h_max, blue_s_min, blue_s_max, blue_v_min, blue_v_max};
    cfg.open_kernel = std::max(1, open_kernel);
    cfg.close_kernel = std::max(1, close_kernel);
    cfg.min_area = static_cast<double>(min_area);
    cfg.max_area = static_cast<double>(std::max(min_area, max_area));
    cfg.min_aspect = min_aspect_x100 / 100.0;
    cfg.max_aspect = std::max(min_aspect_x100, max_aspect_x100) / 100.0;
    return cfg;
  }
};
}  // namespace

int main(int argc, char ** argv) {
  std::signal(SIGINT, signalHandler);
  std::signal(SIGTERM, signalHandler);

  std::string config_path = defaultConfigPath(argv);
  if (argc > 1) config_path = argv[1];

  laser_aimer::LaserAimerConfig cfg;
  if (!laser_aimer::loadConfig(config_path, &cfg)) {
    return 1;
  }

  try {
    auto provider = std::make_shared<laser_aimer::LaserAimerProvider>(cfg);
    auto system = std::make_shared<laser_aimer::LaserAimerSystem>(cfg);
    auto selector = std::make_shared<laser_aimer::LaserAimerSelector>();
    auto solver = std::make_shared<laser_aimer::LaserAimerSolver>(cfg);
    auto actuator = std::make_shared<laser_aimer::LaserAimerActuator>(cfg);

    aim::Runtime<laser_aimer::LaserAimerInput, laser_aimer::LaserAimerState> runtime(
      provider, system, selector, solver, actuator, cfg.loop_rate_hz);

    FixedTargetTrackbars fixed_target_tuner;
    if (cfg.show_debug) {
      fixed_target_tuner.create(cfg.fixed_target);
      provider->updateFixedTargetConfig(fixed_target_tuner.read());
    }

    runtime.start();
    std::cout << "[LaserAimer] Running. Press Ctrl+C to stop.\n";

    while (!g_stop.load()) {
      if (cfg.show_debug) {
        provider->updateFixedTargetConfig(fixed_target_tuner.read());
        auto images = aim::DebugContext::getInstance().getImages();
        std::vector<std::string> names;
        names.reserve(images.size());
        for (const auto & item : images) names.push_back(item.first);
        std::sort(names.begin(), names.end());
        for (const auto & name : names) {
          const auto it = images.find(name);
          if (it != images.end() && !it->second.empty()) {
            cv::imshow(name, it->second);
          }
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 27) g_stop.store(true);
        if (images.empty()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }

    runtime.stop();
  } catch (const std::exception & e) {
    std::cerr << "[LaserAimer] Fatal: " << e.what() << "\n";
    return 1;
  }

  return 0;
}
