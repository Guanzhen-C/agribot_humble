#include <MvCameraControl.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <agribot_time_sync/affine_clock_mapper.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace
{
std::string error_code(int code)
{
  std::ostringstream stream;
  stream << "0x" << std::hex << std::uppercase << static_cast<uint32_t>(code);
  return stream.str();
}

std::string usb_string(const unsigned char * value)
{
  const auto * text = reinterpret_cast<const char *>(value);
  return std::string(text, strnlen(text, INFO_MAX_BUFFER_SIZE));
}

diagnostic_msgs::msg::KeyValue diagnostic_value(
  const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue result;
  result.key = key;
  result.value = value;
  return result;
}
}  // namespace

class HikrobotMvsCameraNode : public rclcpp::Node
{
public:
  HikrobotMvsCameraNode()
  : Node("hikrobot_mvs_camera"), sdk_initialized_(false), handle_(nullptr), running_(false)
  {
    serial_number_ = declare_parameter<std::string>("serial_number", "");
    frame_id_ = declare_parameter<std::string>("frame_id", "right_camera_optical_frame");
    const auto image_topic =
      declare_parameter<std::string>("image_topic", "/camera/rgb/image_raw");
    const auto camera_info_topic =
      declare_parameter<std::string>("camera_info_topic", "/camera/rgb/camera_info");
    const auto camera_name =
      declare_parameter<std::string>("camera_name", "agribot_hikrobot_right_camera");
    const auto camera_info_url = declare_parameter<std::string>("camera_info_url", "");
    trigger_enable_ = declare_parameter<bool>("trigger_enable", false);
    trigger_selector_ = declare_parameter<std::string>("trigger_selector", "FrameStart");
    trigger_source_ = declare_parameter<std::string>("trigger_source", "Line0");
    trigger_activation_ = declare_parameter<std::string>("trigger_activation", "RisingEdge");
    frame_rate_ = declare_parameter<double>("acquisition_frame_rate", 10.0);
    exposure_auto_ = declare_parameter<std::string>("exposure_auto", "Continuous");
    exposure_time_us_ = declare_parameter<double>("exposure_time_us", 5000.0);
    gain_auto_ = declare_parameter<std::string>("gain_auto", "Continuous");
    gain_ = declare_parameter<double>("gain", 0.0);
    gamma_enable_ = declare_parameter<bool>("gamma_enable", true);
    gamma_ = declare_parameter<double>("gamma", 0.7);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "BayerGB12Packed");
    sdk_buffer_count_ = static_cast<int>(std::clamp<int64_t>(
        declare_parameter<int>("sdk_buffer_count", 2), 1, 8));
    grab_timeout_ms_ = static_cast<int>(std::max<int64_t>(
        declare_parameter<int>("grab_timeout_ms", 1000), 10));
    timestamp_source_ = declare_parameter<std::string>("timestamp_source", "device");
    if (timestamp_source_ != "device" && timestamp_source_ != "receipt") {
      throw std::invalid_argument("timestamp_source must be device or receipt");
    }
    device_timestamp_frequency_hz_ =
      declare_parameter<double>("device_timestamp_frequency_hz", 100000000.0);
    if (!std::isfinite(device_timestamp_frequency_hz_) ||
      device_timestamp_frequency_hz_ <= 0.0)
    {
      throw std::invalid_argument("device_timestamp_frequency_hz must be positive");
    }
    timestamp_offset_sec_ = declare_parameter<double>("timestamp_offset_sec", 0.0);
    agribot_time_sync::ClockMapperConfig clock_config;
    clock_config.window_size = static_cast<std::size_t>(std::max<int64_t>(
        declare_parameter<int>("time_sync_window_size", 600), 20));
    clock_config.min_samples = static_cast<std::size_t>(std::max<int64_t>(
        declare_parameter<int>("time_sync_min_samples", 30), 2));
    clock_config.fit_interval_samples = static_cast<std::size_t>(std::max<int64_t>(
        declare_parameter<int>("time_sync_fit_interval_samples", 5), 1));
    clock_config.min_span_sec = declare_parameter<double>("time_sync_min_span_sec", 3.0);
    clock_config.max_scale_error_ppm =
      declare_parameter<double>("time_sync_max_scale_error_ppm", 1000.0);
    clock_config.reset_threshold_sec =
      declare_parameter<double>("time_sync_reset_threshold_sec", 0.5);
    clock_config.max_device_gap_sec =
      declare_parameter<double>("time_sync_max_device_gap_sec", 2.0);
    clock_mapper_ = std::make_unique<agribot_time_sync::AffineClockMapper>(clock_config);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic, sensor_qos);
    camera_info_pub_ =
      create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, sensor_qos);
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    connected_pub_ = create_publisher<std_msgs::msg::Bool>("~/connected", state_qos);
    frame_rate_pub_ = create_publisher<std_msgs::msg::Float32>("~/frame_rate_hz", state_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::SystemDefaultsQoS());
    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this, camera_name, camera_info_url);

    try {
      open_camera();
    } catch (...) {
      close_camera();
      throw;
    }
    publish_connected(true);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&HikrobotMvsCameraNode::publish_diagnostics, this));
    running_.store(true);
    grab_thread_ = std::thread(&HikrobotMvsCameraNode::grab_loop, this);
  }

  ~HikrobotMvsCameraNode() override
  {
    running_.store(false);
    if (grab_thread_.joinable()) {
      grab_thread_.join();
    }
    close_camera();
  }

private:
  void require_ok(int result, const std::string & operation)
  {
    if (result != MV_OK) {
      throw std::runtime_error(operation + "失败: " + error_code(result));
    }
  }

  void warn_optional(int result, const std::string & operation)
  {
    if (result != MV_OK) {
      RCLCPP_WARN(get_logger(), "%s未生效: %s", operation.c_str(), error_code(result).c_str());
    }
  }

  void open_camera()
  {
    require_ok(MV_CC_Initialize(), "初始化MVS SDK");
    sdk_initialized_ = true;

    MV_CC_DEVICE_INFO_LIST devices{};
    require_ok(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "枚举USB相机");
    if (devices.nDeviceNum == 0) {
      throw std::runtime_error("没有发现海康MVS USB相机");
    }

    MV_CC_DEVICE_INFO * selected = nullptr;
    std::string selected_serial;
    std::string selected_model;
    for (unsigned int i = 0; i < devices.nDeviceNum; ++i) {
      auto * device = devices.pDeviceInfo[i];
      if (device == nullptr || device->nTLayerType != MV_USB_DEVICE) {
        continue;
      }
      const auto serial = usb_string(device->SpecialInfo.stUsb3VInfo.chSerialNumber);
      const auto model = usb_string(device->SpecialInfo.stUsb3VInfo.chModelName);
      RCLCPP_INFO(get_logger(), "发现MVS相机: model=%s serial=%s", model.c_str(), serial.c_str());
      if (serial_number_.empty() || serial == serial_number_) {
        selected = device;
        selected_serial = serial;
        selected_model = model;
        break;
      }
    }
    if (selected == nullptr) {
      throw std::runtime_error("找不到指定序列号的MVS相机: " + serial_number_);
    }

    require_ok(MV_CC_CreateHandle(&handle_, selected), "创建相机句柄");
    require_ok(MV_CC_OpenDevice(handle_), "打开相机");
    require_ok(
      MV_CC_SetImageNodeNum(handle_, static_cast<unsigned int>(sdk_buffer_count_)),
      "设置SDK缓存数量");
    require_ok(
      MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly),
      "设置最新帧抓取策略");

    require_ok(
      MV_CC_SetEnumValueByString(handle_, "PixelFormat", pixel_format_.c_str()),
      "设置像素格式");
    warn_optional(
      MV_CC_SetEnumValueByString(handle_, "ExposureAuto", exposure_auto_.c_str()),
      "设置自动曝光");
    if (exposure_auto_ == "Off") {
      warn_optional(
        MV_CC_SetFloatValue(handle_, "ExposureTime", static_cast<float>(exposure_time_us_)),
        "设置曝光时间");
    }
    warn_optional(
      MV_CC_SetEnumValueByString(handle_, "GainAuto", gain_auto_.c_str()),
      "设置自动增益");
    if (gain_auto_ == "Off") {
      warn_optional(
        MV_CC_SetFloatValue(handle_, "Gain", static_cast<float>(gain_)), "设置增益");
    }
    warn_optional(MV_CC_SetBoolValue(handle_, "GammaEnable", gamma_enable_), "设置Gamma开关");
    if (gamma_enable_) {
      warn_optional(
        MV_CC_SetFloatValue(handle_, "Gamma", static_cast<float>(gamma_)), "设置Gamma");
    }

    if (trigger_enable_) {
      require_ok(
        MV_CC_SetEnumValueByString(handle_, "TriggerSelector", trigger_selector_.c_str()),
        "设置触发选择器");
      require_ok(MV_CC_SetEnumValueByString(handle_, "TriggerMode", "On"), "开启硬触发");
      require_ok(
        MV_CC_SetEnumValueByString(handle_, "TriggerSource", trigger_source_.c_str()),
        "设置触发源");
      require_ok(
        MV_CC_SetEnumValueByString(handle_, "TriggerActivation", trigger_activation_.c_str()),
        "设置触发沿");
    } else {
      require_ok(MV_CC_SetEnumValueByString(handle_, "TriggerMode", "Off"), "关闭触发模式");
      warn_optional(
        MV_CC_SetBoolValue(handle_, "AcquisitionFrameRateEnable", true), "开启帧率限制");
      warn_optional(
        MV_CC_SetFloatValue(handle_, "AcquisitionFrameRate", static_cast<float>(frame_rate_)),
        "设置采集帧率");
    }

    require_ok(MV_CC_StartGrabbing(handle_), "开始取流");
    RCLCPP_INFO(
      get_logger(), "相机已启动: model=%s serial=%s mode=%s target=%.1fHz buffers=%d",
      selected_model.c_str(), selected_serial.c_str(), trigger_enable_ ? "trigger" : "free-run",
      frame_rate_, sdk_buffer_count_);
  }

  void close_camera() noexcept
  {
    publish_connected(false);
    if (handle_ != nullptr) {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
    if (sdk_initialized_) {
      MV_CC_Finalize();
      sdk_initialized_ = false;
    }
  }

  void publish_connected(bool connected)
  {
    if (connected_pub_) {
      std_msgs::msg::Bool message;
      message.data = connected;
      connected_pub_->publish(message);
    }
  }

  rclcpp::Time frame_stamp(const MV_FRAME_OUT_INFO_EX & frame_info)
  {
    const rclcpp::Time ros_receipt = now();
    double receipt_sec = ros_receipt.seconds();
    if (frame_info.nHostTimeStamp > 0) {
      const double sdk_receipt_sec = static_cast<double>(frame_info.nHostTimeStamp) * 1.0e-3;
      if (std::abs(sdk_receipt_sec - receipt_sec) < 5.0) {
        receipt_sec = sdk_receipt_sec;
      }
    }

    double stamp_sec = receipt_sec;
    const std::uint64_t device_ticks =
      (static_cast<std::uint64_t>(frame_info.nDevTimeStampHigh) << 32U) |
      static_cast<std::uint64_t>(frame_info.nDevTimeStampLow);
    std::lock_guard<std::mutex> lock(clock_mutex_);
    if (timestamp_source_ == "device" && device_ticks != 0U) {
      last_clock_result_ = clock_mapper_->observe(
        static_cast<double>(device_ticks) / device_timestamp_frequency_hz_, receipt_sec);
      stamp_sec = last_clock_result_.stamp_sec;
      device_timestamp_valid_ = true;
    } else {
      device_timestamp_valid_ = timestamp_source_ == "receipt";
    }
    stamp_sec += timestamp_offset_sec_;
    return rclcpp::Time(
      static_cast<std::int64_t>(stamp_sec * 1.0e9), ros_receipt.get_clock_type());
  }

  void publish_frame(const MV_FRAME_OUT & frame)
  {
    const auto stamp = frame_stamp(frame.stFrameInfo);
    const uint32_t width = frame.stFrameInfo.nExtendWidth;
    const uint32_t height = frame.stFrameInfo.nExtendHeight;
    if (width == 0 || height == 0) {
      return;
    }

    const auto required_size = static_cast<size_t>(width) * height * 3U;
    converted_buffer_.resize(required_size);
    MV_CC_PIXEL_CONVERT_PARAM_EX conversion{};
    conversion.nWidth = width;
    conversion.nHeight = height;
    conversion.enSrcPixelType = frame.stFrameInfo.enPixelType;
    conversion.pSrcData = frame.pBufAddr;
    conversion.nSrcDataLen = frame.stFrameInfo.nFrameLen;
    conversion.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
    conversion.pDstBuffer = converted_buffer_.data();
    conversion.nDstBufferSize = static_cast<unsigned int>(converted_buffer_.size());
    const int result = MV_CC_ConvertPixelTypeEx(handle_, &conversion);
    if (result != MV_OK) {
      ++conversion_errors_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "像素转换失败: %s", error_code(result).c_str());
      return;
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = stamp;
    image.header.frame_id = frame_id_;
    image.height = height;
    image.width = width;
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = width * 3U;
    image.data.assign(converted_buffer_.begin(), converted_buffer_.begin() + conversion.nDstLen);
    image_pub_->publish(std::move(image));

    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = stamp;
    camera_info.header.frame_id = frame_id_;
    camera_info.width = width;
    camera_info.height = height;
    camera_info_pub_->publish(std::move(camera_info));
    ++published_frames_;
  }

  void publish_diagnostics()
  {
    agribot_time_sync::ClockMapperResult clock_result;
    bool device_timestamp_valid = false;
    {
      std::lock_guard<std::mutex> lock(clock_mutex_);
      clock_result = last_clock_result_;
      device_timestamp_valid = device_timestamp_valid_;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "hikrobot_mvs/time_sync";
    status.hardware_id = serial_number_;
    if (timestamp_source_ == "receipt") {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "using SDK receipt timestamps";
    } else if (!device_timestamp_valid) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "camera device timestamp unavailable";
    } else if (!clock_result.synchronized) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "device clock mapping warming up";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "device clock mapped to ROS time";
    }
    status.values.push_back(diagnostic_value("source", timestamp_source_));
    status.values.push_back(
      diagnostic_value("capture_mode", trigger_enable_ ? "hardware_trigger" : "free_run"));
    status.values.push_back(diagnostic_value("trigger_selector", trigger_selector_));
    status.values.push_back(diagnostic_value("trigger_source", trigger_source_));
    status.values.push_back(diagnostic_value("trigger_activation", trigger_activation_));
    status.values.push_back(
      diagnostic_value(
        "device_tick_hz", std::to_string(device_timestamp_frequency_hz_)));
    status.values.push_back(
      diagnostic_value(
        "synchronized", clock_result.synchronized ? "true" : "false"));
    status.values.push_back(
      diagnostic_value(
        "samples", std::to_string(clock_result.sample_count)));
    status.values.push_back(
      diagnostic_value(
        "scale_error_ppm", std::to_string(clock_result.scale_error_ppm)));
    status.values.push_back(
      diagnostic_value(
        "estimated_delay_ms", std::to_string(clock_result.estimated_delay_sec * 1.0e3)));
    status.values.push_back(
      diagnostic_value(
        "delay_jitter_ms", std::to_string(clock_result.delay_jitter_sec * 1.0e3)));
    status.values.push_back(
      diagnostic_value(
        "reset_count", std::to_string(clock_result.reset_count)));
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(std::move(array));
  }

  void grab_loop()
  {
    auto last_report = std::chrono::steady_clock::now();
    uint64_t last_count = 0;
    while (running_.load() && rclcpp::ok()) {
      MV_FRAME_OUT frame{};
      const int result = MV_CC_GetImageBuffer(handle_, &frame, grab_timeout_ms_);
      if (result == MV_OK) {
        try {
          publish_frame(frame);
        } catch (const std::exception & error) {
          RCLCPP_ERROR(get_logger(), "发布相机帧失败: %s", error.what());
        }
        MV_CC_FreeImageBuffer(handle_, &frame);
      } else {
        ++grab_timeouts_;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "等待相机帧超时: %s", error_code(result).c_str());
      }

      const auto current = std::chrono::steady_clock::now();
      const double elapsed = std::chrono::duration<double>(current - last_report).count();
      if (elapsed >= 5.0) {
        const uint64_t count = published_frames_.load();
        const float measured_rate = static_cast<float>((count - last_count) / elapsed);
        std_msgs::msg::Float32 rate_message;
        rate_message.data = measured_rate;
        frame_rate_pub_->publish(rate_message);
        RCLCPP_INFO(
          get_logger(), "相机取流 %.2f Hz, published=%lu, timeouts=%lu, convert_errors=%lu",
          measured_rate, count, grab_timeouts_.load(), conversion_errors_.load());
        last_count = count;
        last_report = current;
      }
    }
  }

  std::string serial_number_;
  std::string frame_id_;
  std::string trigger_selector_;
  std::string trigger_source_;
  std::string trigger_activation_;
  std::string exposure_auto_;
  std::string gain_auto_;
  std::string pixel_format_;
  std::string timestamp_source_;
  bool trigger_enable_;
  bool gamma_enable_;
  double frame_rate_;
  double exposure_time_us_;
  double gain_;
  double gamma_;
  double device_timestamp_frequency_hz_;
  double timestamp_offset_sec_;
  int sdk_buffer_count_;
  int grab_timeout_ms_;

  bool sdk_initialized_;
  void * handle_;
  std::atomic<bool> running_;
  std::thread grab_thread_;
  std::vector<unsigned char> converted_buffer_;
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> grab_timeouts_{0};
  std::atomic<uint64_t> conversion_errors_{0};
  std::mutex clock_mutex_;
  std::unique_ptr<agribot_time_sync::AffineClockMapper> clock_mapper_;
  agribot_time_sync::ClockMapperResult last_clock_result_;
  bool device_timestamp_valid_{false};

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr frame_rate_pub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<HikrobotMvsCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("hikrobot_mvs_camera"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
