#include "LaserAimerSystem.hpp"

#include "../../include/core/DebugContext.hpp"

#include <algorithm>
#include <cmath>

namespace laser_aimer {
namespace {

double clampDtSeconds(double dt_ms, double reset_dt_ms) {
  if (dt_ms <= 0.0 || dt_ms > reset_dt_ms) return -1.0;
  return std::clamp(dt_ms / 1000.0, 0.001, 0.1);
}

}  // namespace

LaserAimerSystem::LaserAimerSystem(const LaserAimerConfig & cfg) : cfg_(cfg) {}

void LaserAimerSystem::resetTargetFilter() {
  target_filter_initialized_ = false;
  target_filter_last_ts_ = 0;
}

TargetMeasurement LaserAimerSystem::filterMeasurement(const TargetMeasurement & meas) {
  if (!cfg_.control.use_target_kalman) {
    aim::DebugContext::getInstance().setPoint("Kalman Predict", {}, false);
    return meas;
  }

  TargetMeasurement out = meas;
  const double reset_dt = std::max(1.0, cfg_.control.target_kalman_reset_dt_ms);
  const double dt = target_filter_initialized_ ?
    clampDtSeconds(static_cast<double>(meas.timestamp_ms - target_filter_last_ts_), reset_dt) : -1.0;

  if (!target_filter_initialized_ || dt < 0.0) {
    target_filter_ = cv::KalmanFilter(6, 2, 0, CV_32F);
    cv::setIdentity(target_filter_.transitionMatrix);
    target_filter_.measurementMatrix = cv::Mat::zeros(2, 6, CV_32F);
    target_filter_.measurementMatrix.at<float>(0, 0) = 1.0F;
    target_filter_.measurementMatrix.at<float>(1, 1) = 1.0F;
    cv::setIdentity(target_filter_.processNoiseCov,
                    cv::Scalar(static_cast<float>(cfg_.control.target_kalman_accel_noise)));
    cv::setIdentity(target_filter_.measurementNoiseCov,
                    cv::Scalar(static_cast<float>(cfg_.control.target_kalman_meas_noise *
                                                  cfg_.control.target_kalman_meas_noise)));
    cv::setIdentity(target_filter_.errorCovPost, cv::Scalar(25.0));
    target_filter_.statePost.at<float>(0) = meas.uv.x;
    target_filter_.statePost.at<float>(1) = meas.uv.y;
    target_filter_.statePost.at<float>(2) = 0.0F;
    target_filter_.statePost.at<float>(3) = 0.0F;
    target_filter_.statePost.at<float>(4) = 0.0F;
    target_filter_.statePost.at<float>(5) = 0.0F;
    target_filter_initialized_ = true;
    target_filter_last_ts_ = meas.timestamp_ms;
    aim::DebugContext::getInstance().setPoint("Kalman Predict", meas.uv, true);
    return out;
  }

  target_filter_.transitionMatrix.at<float>(0, 2) = static_cast<float>(dt);
  target_filter_.transitionMatrix.at<float>(1, 3) = static_cast<float>(dt);
  target_filter_.transitionMatrix.at<float>(0, 4) = static_cast<float>(0.5 * dt * dt);
  target_filter_.transitionMatrix.at<float>(1, 5) = static_cast<float>(0.5 * dt * dt);
  target_filter_.transitionMatrix.at<float>(2, 4) = static_cast<float>(dt);
  target_filter_.transitionMatrix.at<float>(3, 5) = static_cast<float>(dt);

  const float q = static_cast<float>(std::max(1e-6, cfg_.control.target_kalman_accel_noise));
  target_filter_.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);
  target_filter_.processNoiseCov.at<float>(0, 0) = q * static_cast<float>(dt * dt * dt * dt);
  target_filter_.processNoiseCov.at<float>(1, 1) = q * static_cast<float>(dt * dt * dt * dt);
  target_filter_.processNoiseCov.at<float>(2, 2) = q * static_cast<float>(dt * dt);
  target_filter_.processNoiseCov.at<float>(3, 3) = q * static_cast<float>(dt * dt);
  target_filter_.processNoiseCov.at<float>(4, 4) = q;
  target_filter_.processNoiseCov.at<float>(5, 5) = q;

  const cv::Mat predicted = target_filter_.predict();
  aim::DebugContext::getInstance().setPoint(
    "Kalman Predict",
    cv::Point2f(predicted.at<float>(0), predicted.at<float>(1)),
    true);
  cv::Mat measurement(2, 1, CV_32F);
  measurement.at<float>(0) = meas.uv.x;
  measurement.at<float>(1) = meas.uv.y;
  const cv::Mat corrected = target_filter_.correct(measurement);
  out.uv.x = corrected.at<float>(0);
  out.uv.y = corrected.at<float>(1);
  target_filter_last_ts_ = meas.timestamp_ms;
  return out;
}

void LaserAimerSystem::update(const LaserAimerInput & input) {
  state_.last_input = input;
  state_.measurement = input.measurement;

  if (input.measurement.valid) {
    state_.detect_count++;
    state_.lost_count = 0;
    state_.filtered_measurement = filterMeasurement(input.measurement);
    state_.locked = state_.detect_count >= cfg_.min_detect_count;
    return;
  }

  state_.detect_count = 0;
  state_.lost_count++;
  if (state_.lost_count > cfg_.max_lost_count) {
    state_.locked = false;
    state_.filtered_measurement = {};
    resetTargetFilter();
    aim::DebugContext::getInstance().setPoint("Kalman Predict", {}, false);
  } else {
    state_.filtered_measurement.valid = false;
    state_.filtered_measurement.timestamp_ms = input.timestamp_ms;
    aim::DebugContext::getInstance().setPoint("Kalman Predict", {}, false);
  }
}

void LaserAimerSystem::updateSelfState(const aim::SelfState & self) {
  state_.self = self;
}

}  // namespace laser_aimer
