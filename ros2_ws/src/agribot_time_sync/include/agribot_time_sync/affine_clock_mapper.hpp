// Copyright 2026 cgz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef AGRIBOT_TIME_SYNC__AFFINE_CLOCK_MAPPER_HPP_
#define AGRIBOT_TIME_SYNC__AFFINE_CLOCK_MAPPER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <vector>

namespace agribot_time_sync
{

struct ClockMapperConfig
{
  std::size_t window_size{200};
  std::size_t min_samples{20};
  std::size_t fit_interval_samples{10};
  double min_span_sec{0.5};
  double max_scale_error_ppm{500.0};
  double reset_threshold_sec{0.5};
  double max_device_gap_sec{10.0};
  double transport_delay_sec{0.0};
  double lower_delay_quantile{0.05};
};

struct ClockMapperResult
{
  double stamp_sec{0.0};
  bool synchronized{false};
  std::size_t sample_count{0};
  double scale_error_ppm{0.0};
  double estimated_delay_sec{0.0};
  double delay_jitter_sec{0.0};
  std::uint64_t reset_count{0};
};

class AffineClockMapper
{
public:
  explicit AffineClockMapper(ClockMapperConfig config = {})
  : config_(sanitize(config))
  {
  }

  ClockMapperResult observe(double device_time_sec, double receipt_time_sec)
  {
    if (!std::isfinite(device_time_sec) || !std::isfinite(receipt_time_sec)) {
      return snapshot(receipt_time_sec);
    }

    if (!initialized_) {
      initialize(device_time_sec, receipt_time_sec, false);
    } else {
      const double device_delta = device_time_sec - last_device_time_sec_;
      const double receipt_delta = receipt_time_sec - last_receipt_time_sec_;
      const double predicted = mapUnchecked(device_time_sec);
      const bool device_reset = device_delta <= 0.0 || device_delta > config_.max_device_gap_sec;
      const bool host_clock_reset = receipt_delta < 0.0;
      const bool mapping_jump = synchronized_ &&
        std::abs(receipt_time_sec - predicted) > config_.reset_threshold_sec;
      if (device_reset || host_clock_reset || mapping_jump) {
        initialize(device_time_sec, receipt_time_sec, true);
      }
    }

    const double relative_device = device_time_sec - device_origin_sec_;
    const double relative_receipt = receipt_time_sec - receipt_origin_sec_;
    samples_.push_back({relative_device, relative_receipt});
    while (samples_.size() > config_.window_size) {
      samples_.pop_front();
    }
    ++observations_since_fit_;
    if (!synchronized_ || observations_since_fit_ >= config_.fit_interval_samples) {
      fit();
      observations_since_fit_ = 0;
    }

    double stamp_sec = mapUnchecked(device_time_sec);
    stamp_sec = std::min(stamp_sec, receipt_time_sec);
    if (have_last_stamp_ && stamp_sec <= last_stamp_sec_) {
      stamp_sec = last_stamp_sec_ + 1.0e-9;
    }
    last_stamp_sec_ = stamp_sec;
    have_last_stamp_ = true;
    last_device_time_sec_ = device_time_sec;
    last_receipt_time_sec_ = receipt_time_sec;

    auto result = snapshot(stamp_sec);
    result.estimated_delay_sec = receipt_time_sec - stamp_sec;
    return result;
  }

  void reset()
  {
    initialize(0.0, 0.0, initialized_);
    initialized_ = false;
    samples_.clear();
    have_last_stamp_ = false;
    synchronized_ = false;
  }

  ClockMapperResult status() const
  {
    return snapshot(have_last_stamp_ ? last_stamp_sec_ : 0.0);
  }

private:
  struct Sample
  {
    double device;
    double receipt;
  };

  static ClockMapperConfig sanitize(ClockMapperConfig config)
  {
    config.window_size = std::max<std::size_t>(config.window_size, 4U);
    config.min_samples = std::clamp<std::size_t>(config.min_samples, 2U, config.window_size);
    config.fit_interval_samples = std::max<std::size_t>(config.fit_interval_samples, 1U);
    config.min_span_sec = std::max(config.min_span_sec, 0.0);
    config.max_scale_error_ppm = std::max(config.max_scale_error_ppm, 1.0);
    config.reset_threshold_sec = std::max(config.reset_threshold_sec, 0.01);
    config.max_device_gap_sec = std::max(config.max_device_gap_sec, 0.01);
    config.transport_delay_sec = std::max(config.transport_delay_sec, 0.0);
    config.lower_delay_quantile = std::clamp(config.lower_delay_quantile, 0.0, 0.5);
    return config;
  }

  void initialize(double device_time_sec, double receipt_time_sec, bool count_reset)
  {
    if (count_reset) {
      ++reset_count_;
    }
    initialized_ = true;
    synchronized_ = false;
    samples_.clear();
    device_origin_sec_ = device_time_sec;
    receipt_origin_sec_ = receipt_time_sec;
    last_device_time_sec_ = device_time_sec;
    last_receipt_time_sec_ = receipt_time_sec;
    scale_ = 1.0;
    offset_sec_ = -config_.transport_delay_sec;
    delay_jitter_sec_ = 0.0;
    observations_since_fit_ = 0;
    have_last_stamp_ = false;
  }

  void fit()
  {
    if (samples_.size() < config_.min_samples) {
      return;
    }
    const double span = samples_.back().device - samples_.front().device;
    if (span < config_.min_span_sec) {
      return;
    }

    double mean_device = 0.0;
    double mean_receipt = 0.0;
    for (const auto & sample : samples_) {
      mean_device += sample.device;
      mean_receipt += sample.receipt;
    }
    mean_device /= static_cast<double>(samples_.size());
    mean_receipt /= static_cast<double>(samples_.size());

    double covariance = 0.0;
    double variance = 0.0;
    for (const auto & sample : samples_) {
      const double centered_device = sample.device - mean_device;
      covariance += centered_device * (sample.receipt - mean_receipt);
      variance += centered_device * centered_device;
    }
    if (variance <= std::numeric_limits<double>::epsilon()) {
      return;
    }

    const double scale_limit = config_.max_scale_error_ppm * 1.0e-6;
    scale_ = std::clamp(covariance / variance, 1.0 - scale_limit, 1.0 + scale_limit);

    std::vector<double> offsets;
    offsets.reserve(samples_.size());
    for (const auto & sample : samples_) {
      offsets.push_back(sample.receipt - scale_ * sample.device);
    }
    const auto quantile_index = static_cast<std::size_t>(
      config_.lower_delay_quantile * static_cast<double>(offsets.size() - 1U));
    std::nth_element(
      offsets.begin(), offsets.begin() + static_cast<std::ptrdiff_t>(quantile_index), offsets.end());
    offset_sec_ = offsets[quantile_index] - config_.transport_delay_sec;

    double squared_sum = 0.0;
    for (const double offset : offsets) {
      const double delay = offset - offset_sec_ - config_.transport_delay_sec;
      squared_sum += delay * delay;
    }
    delay_jitter_sec_ = std::sqrt(squared_sum / static_cast<double>(offsets.size()));
    synchronized_ = true;
  }

  double mapUnchecked(double device_time_sec) const
  {
    return receipt_origin_sec_ +
           scale_ * (device_time_sec - device_origin_sec_) + offset_sec_;
  }

  ClockMapperResult snapshot(double stamp_sec) const
  {
    ClockMapperResult result;
    result.stamp_sec = stamp_sec;
    result.synchronized = synchronized_;
    result.sample_count = samples_.size();
    result.scale_error_ppm = (scale_ - 1.0) * 1.0e6;
    result.delay_jitter_sec = delay_jitter_sec_;
    result.reset_count = reset_count_;
    return result;
  }

  ClockMapperConfig config_;
  std::deque<Sample> samples_;
  bool initialized_{false};
  bool synchronized_{false};
  bool have_last_stamp_{false};
  double device_origin_sec_{0.0};
  double receipt_origin_sec_{0.0};
  double last_device_time_sec_{0.0};
  double last_receipt_time_sec_{0.0};
  double last_stamp_sec_{0.0};
  double scale_{1.0};
  double offset_sec_{0.0};
  double delay_jitter_sec_{0.0};
  std::uint64_t reset_count_{0};
  std::size_t observations_since_fit_{0};
};

class WrappingCounter32
{
public:
  std::uint64_t unwrap(std::uint32_t value, bool & reset_detected)
  {
    reset_detected = false;
    if (!initialized_) {
      initialized_ = true;
      last_value_ = value;
      return value;
    }
    if (value < last_value_) {
      const std::uint32_t backwards = last_value_ - value;
      if (backwards > (std::numeric_limits<std::uint32_t>::max() / 2U)) {
        ++wrap_count_;
      } else {
        reset_detected = true;
        wrap_count_ = 0;
      }
    }
    last_value_ = value;
    return (wrap_count_ << 32U) | static_cast<std::uint64_t>(value);
  }

  void reset()
  {
    initialized_ = false;
    last_value_ = 0;
    wrap_count_ = 0;
  }

private:
  bool initialized_{false};
  std::uint32_t last_value_{0};
  std::uint64_t wrap_count_{0};
};

}  // namespace agribot_time_sync

#endif  // AGRIBOT_TIME_SYNC__AFFINE_CLOCK_MAPPER_HPP_
