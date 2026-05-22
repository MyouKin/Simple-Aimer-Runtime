#include "DroneDetector.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <iostream>
#include <opencv2/imgproc.hpp>

#ifdef LASER_AIMER_WITH_TRT
#include "api.h"
#endif

namespace laser_aimer {
namespace {

std::string toLower(std::string v) {
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return v;
}

std::string derivedEnginePath(const DroneDetectorConfig & cfg) {
  if (!cfg.engine_path.empty()) return cfg.engine_path;
  if (cfg.model_path.empty()) return {};
  std::filesystem::path path(cfg.model_path);
  path.replace_extension(".engine");
  return path.string();
}

int64_t steadyNowMs() {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
}

#ifdef LASER_AIMER_WITH_TRT
TRTInferX::PreprocessMode parsePreprocess(const std::string & value) {
  return toLower(value) == "resize" ? TRTInferX::PreprocessMode::RESIZE
                                    : TRTInferX::PreprocessMode::LETTERBOX;
}

TRTInferX::OutputMode parseOutputMode(const std::string & value) {
  const std::string mode = toLower(value);
  if (mode == "packed_nms") return TRTInferX::OutputMode::PACKED_NMS;
  if (mode == "raw_with_nms") return TRTInferX::OutputMode::RAW_WITH_NMS;
  if (mode == "raw_only") return TRTInferX::OutputMode::RAW_ONLY;
  return TRTInferX::OutputMode::AUTO;
}
#endif

}  // namespace

struct DroneDetector::TrtImpl {
#ifdef LASER_AIMER_WITH_TRT
  TRTInferX::Api api;
  TRTInferX::EngineConfig engine_cfg;
  TRTInferX::InferOptions infer_opt;
#endif
  bool loaded = false;
};

DroneDetector::DroneDetector(const DroneDetectorConfig & cfg) : cfg_(cfg) {
  if (cfg_.backend == "trt_engine") {
    tryLoadTrtEngine();
  } else if (cfg_.backend == "opencv_dnn") {
    tryLoadOpenCVDnn();
  } else if (cfg_.backend != "full_frame" && cfg_.backend != "disabled") {
    std::cerr << "[LaserAimer] Unknown drone detector backend '" << cfg_.backend
              << "', falling back to full_frame.\n";
    cfg_.backend = "full_frame";
  }
}

DroneDetector::~DroneDetector() = default;

std::vector<DroneBox> DroneDetector::detect(const cv::Mat & bgr) {
  if (bgr.empty() || cfg_.backend == "disabled") return {};
  if (cfg_.backend == "full_frame") return fullFrame(bgr);
  if (cfg_.backend == "trt_engine") {
    if (trt_ && trt_->loaded) return detectTrtEngine(bgr);
    return fullFrame(bgr);
  }
  if (cfg_.backend == "opencv_dnn" && !net_loaded_) {
    return fullFrame(bgr);
  }
  return detectOpenCVDnn(bgr);
}

std::vector<DroneBox> DroneDetector::fullFrame(const cv::Mat & bgr) const {
  DroneBox box;
  box.valid = true;
  box.box = cv::Rect(0, 0, bgr.cols, bgr.rows);
  box.center = cv::Point2f(bgr.cols * 0.5F, bgr.rows * 0.5F);
  box.confidence = 1.0F;
  box.label = "full_frame";
  return {box};
}

void DroneDetector::tryLoadOpenCVDnn() {
  if (cfg_.model_path.empty()) {
    std::cerr << "[LaserAimer] drone_model_path is empty; falling back to full_frame.\n";
    return;
  }
  try {
    net_ = cv::dnn::readNet(cfg_.model_path);
    net_loaded_ = !net_.empty();
    if (net_loaded_) {
      std::cout << "[LaserAimer] Loaded OpenCV DNN drone model: " << cfg_.model_path << "\n";
    }
  } catch (const cv::Exception & e) {
    std::cerr << "[LaserAimer] Failed to load OpenCV DNN model: " << e.what()
              << "\n[LaserAimer] OpenCV DNN does not load Ultralytics .pt directly; "
              << "falling back to full_frame.\n";
  }
}

void DroneDetector::tryLoadTrtEngine() {
  trt_ = std::make_unique<TrtImpl>();
  const std::string engine_path = derivedEnginePath(cfg_);
  if (engine_path.empty()) {
    std::cerr << "[LaserAimer] drone_engine_path is empty; falling back to full_frame.\n";
    return;
  }
  if (!std::filesystem::exists(engine_path)) {
    std::cerr << "[LaserAimer] TRT engine not found: " << engine_path
              << "\n[LaserAimer] Falling back to full_frame until the .engine file is generated.\n";
    return;
  }

#ifdef LASER_AIMER_WITH_TRT
  auto & engine_cfg = trt_->engine_cfg;
  auto & infer_opt = trt_->infer_opt;
  engine_cfg.engine_path = engine_path;
  engine_cfg.device = cfg_.device_id;
  engine_cfg.target_w = cfg_.input_width;
  engine_cfg.target_h = cfg_.input_height;
  engine_cfg.max_batch = std::max(1, cfg_.max_batch);
  engine_cfg.streams = std::max(1, cfg_.streams);
  engine_cfg.auto_streams = cfg_.auto_streams;
  engine_cfg.num_classes = std::max(1, cfg_.num_classes);
  engine_cfg.prep = parsePreprocess(cfg_.preprocess);
  engine_cfg.out_mode = parseOutputMode(cfg_.output_mode);
  engine_cfg.nms_score = cfg_.conf_threshold;
  engine_cfg.nms_iou = cfg_.nms_threshold;

  infer_opt.conf = cfg_.conf_threshold;
  infer_opt.iou = cfg_.nms_threshold;
  infer_opt.max_det = std::max(1, cfg_.max_det);
  infer_opt.apply_sigmoid = cfg_.apply_sigmoid;
  infer_opt.box_fmt = toLower(cfg_.box_format) == "xyxy" ? 1 : 0;

  if (!trt_->api.load(engine_cfg)) {
    std::cerr << "[LaserAimer] Failed to load TRT engine: " << engine_path
              << "\n[LaserAimer] Falling back to full_frame.\n";
    return;
  }
  trt_->loaded = true;
  std::cout << "[LaserAimer] Loaded TRT drone engine: " << engine_path << "\n";
#else
  std::cerr << "[LaserAimer] TRT engine exists but this binary was built without "
            << "LASER_AIMER_WITH_TRT; falling back to full_frame: " << engine_path << "\n";
#endif
}

std::vector<DroneBox> DroneDetector::detectOpenCVDnn(const cv::Mat & bgr) {
  cv::Mat blob = cv::dnn::blobFromImage(
    bgr, 1.0 / 255.0, cv::Size(cfg_.input_width, cfg_.input_height),
    cv::Scalar(), true, false);
  net_.setInput(blob);

  std::vector<cv::Mat> outputs;
  net_.forward(outputs, net_.getUnconnectedOutLayersNames());
  if (outputs.empty()) return {};

  cv::Mat out = outputs.front();
  if (out.dims == 3 && out.size[1] < out.size[2]) {
    out = out.reshape(1, out.size[1]);
    cv::transpose(out, out);
  } else if (out.dims == 3) {
    out = out.reshape(1, out.size[1]);
  } else if (out.dims == 2) {
    out = out.reshape(1, out.rows);
  } else {
    return {};
  }

  std::vector<cv::Rect> boxes;
  std::vector<float> scores;
  const float x_scale = static_cast<float>(bgr.cols) / static_cast<float>(cfg_.input_width);
  const float y_scale = static_cast<float>(bgr.rows) / static_cast<float>(cfg_.input_height);

  for (int i = 0; i < out.rows; ++i) {
    const float * row = out.ptr<float>(i);
    float score = 0.0F;
    if (out.cols >= 6) {
      score = row[4];
      for (int c = 5; c < out.cols; ++c) score = std::max(score, row[c]);
    } else if (out.cols >= 5) {
      score = row[4];
    } else {
      continue;
    }
    if (score < cfg_.conf_threshold) continue;

    float cx = row[0];
    float cy = row[1];
    float w = row[2];
    float h = row[3];
    if (cx <= 1.5F && cy <= 1.5F && w <= 1.5F && h <= 1.5F) {
      cx *= static_cast<float>(bgr.cols);
      cy *= static_cast<float>(bgr.rows);
      w *= static_cast<float>(bgr.cols);
      h *= static_cast<float>(bgr.rows);
    } else {
      cx *= x_scale;
      cy *= y_scale;
      w *= x_scale;
      h *= y_scale;
    }

    cv::Rect rect(
      cv::Point(static_cast<int>(cx - w * 0.5F), static_cast<int>(cy - h * 0.5F)),
      cv::Size(static_cast<int>(w), static_cast<int>(h)));
    rect &= cv::Rect(0, 0, bgr.cols, bgr.rows);
    if (rect.area() <= 0) continue;
    boxes.push_back(rect);
    scores.push_back(score);
  }

  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, scores, cfg_.conf_threshold, cfg_.nms_threshold, keep);
  std::vector<DroneBox> result;
  for (int idx : keep) {
    DroneBox d;
    d.valid = true;
    d.box = boxes[idx];
    d.center = cv::Point2f(
      d.box.x + d.box.width * 0.5F,
      d.box.y + d.box.height * 0.5F);
    d.confidence = scores[idx];
    d.label = "drone";
    result.push_back(d);
    if (cfg_.top_k > 0 && static_cast<int>(result.size()) >= cfg_.top_k) break;
  }
  return result;
}

std::vector<DroneBox> DroneDetector::detectTrtEngine(const cv::Mat & bgr) {
#ifdef LASER_AIMER_WITH_TRT
  TRTInferX::ImageInput input{};
  input.mem = TRTInferX::MemoryType::CPU;
  input.data = bgr.data;
  input.width = bgr.cols;
  input.height = bgr.rows;
  input.stride_bytes = static_cast<int>(bgr.step);
  input.timestamp_ms = steadyNowMs();
  input.color = TRTInferX::ColorSpace::BGR;
  input.layout = TRTInferX::Layout::HWC;
  input.dtype = TRTInferX::DType::UINT8;
  input.prep = trt_->engine_cfg.prep;
  input.target_w = trt_->engine_cfg.target_w;
  input.target_h = trt_->engine_cfg.target_h;
  input.device_id = trt_->engine_cfg.device;

  auto batches = trt_->api.infer({input}, trt_->infer_opt);
  if (batches.empty() || batches.front().empty()) return {};

  auto & dets = batches.front();
  std::sort(dets.begin(), dets.end(), [](const TRTInferX::Det & a, const TRTInferX::Det & b) {
    return a.score > b.score;
  });

  std::vector<DroneBox> result;
  const int keep = cfg_.top_k > 0 ? std::min<int>(cfg_.top_k, static_cast<int>(dets.size()))
                                  : static_cast<int>(dets.size());
  result.reserve(static_cast<size_t>(keep));
  const cv::Rect image_rect(0, 0, bgr.cols, bgr.rows);
  for (int i = 0; i < keep; ++i) {
    const auto & det = dets[static_cast<size_t>(i)];
    cv::Rect rect(
      cv::Point(static_cast<int>(det.x1), static_cast<int>(det.y1)),
      cv::Point(static_cast<int>(det.x2), static_cast<int>(det.y2)));
    rect &= image_rect;
    if (rect.area() <= 0) continue;

    DroneBox box;
    box.valid = true;
    box.box = rect;
    box.center = cv::Point2f(
      rect.x + rect.width * 0.5F,
      rect.y + rect.height * 0.5F);
    box.confidence = det.score;
    box.label = "drone";
    result.push_back(box);
  }
  return result;
#else
  (void)bgr;
  return {};
#endif
}

}  // namespace laser_aimer
