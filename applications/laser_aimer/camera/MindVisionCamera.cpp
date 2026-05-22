#include "MindVisionCamera.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

#include "CameraApi.h"

#include <iostream>
#include <stdexcept>

namespace laser_aimer {

MindVisionCamera::MindVisionCamera(const CameraConfig & cfg) : cfg_(cfg) {
  CameraSdkInit(1);

  int camera_num = 16;
  tSdkCameraDevInfo camera_info_list[16]{};
  if (CameraEnumerateDevice(camera_info_list, &camera_num) != CAMERA_STATUS_SUCCESS ||
      camera_num <= 0) {
    throw std::runtime_error("MindVision camera not found");
  }

  if (CameraInit(&camera_info_list[0], -1, -1, &handle_) != CAMERA_STATUS_SUCCESS) {
    throw std::runtime_error("failed to initialize MindVision camera");
  }

  tSdkCameraCapbility cap{};
  CameraGetCapability(handle_, &cap);
  width_ = cap.sResolutionRange.iWidthMax;
  height_ = cap.sResolutionRange.iHeightMax;

  CameraSetAeState(handle_, FALSE);
  CameraSetExposureTime(handle_, cfg_.exposure_ms * 1000.0);
  CameraSetAnalogGainX(handle_, static_cast<float>(cfg_.analog_gain));
  CameraSetGamma(handle_, static_cast<int>(cfg_.gamma * 100.0));
  CameraSetIspOutFormat(handle_, CAMERA_MEDIA_TYPE_BGR8);
  CameraSetTriggerMode(handle_, 0);
  CameraSetFrameSpeed(handle_, 1);

  if (CameraPlay(handle_) != CAMERA_STATUS_SUCCESS) {
    CameraUnInit(handle_);
    handle_ = -1;
    throw std::runtime_error("failed to start MindVision camera");
  }

  opened_ = true;
  std::cout << "[LaserAimer] MindVision camera opened: " << width_ << "x" << height_ << "\n";
}

MindVisionCamera::~MindVisionCamera() {
  if (opened_ && handle_ != -1) {
    CameraUnInit(handle_);
  }
}

bool MindVisionCamera::read(cv::Mat & frame, SteadyTimePoint & timestamp) {
  if (!opened_) return false;
  tSdkFrameHead head{};
  BYTE * raw = nullptr;
  const int status = CameraGetImageBuffer(handle_, &head, &raw, cfg_.timeout_ms);
  if (status != CAMERA_STATUS_SUCCESS) return false;

  cv::Mat image(head.iHeight, head.iWidth, CV_8UC3);
  CameraImageProcess(handle_, raw, image.data, &head);
  CameraReleaseImageBuffer(handle_, raw);
  if (cfg_.flip_code != 2) cv::flip(image, image, cfg_.flip_code);
  frame = image;
  timestamp = std::chrono::steady_clock::now();
  return true;
}

}  // namespace laser_aimer
