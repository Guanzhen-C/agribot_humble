#include <fcntl.h>
#include <linux/pps.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
constexpr std::size_t kChannelCount = 4;
constexpr std::uint32_t kHardwareTriggerMode = 1;

struct LpwmChannelAttributes
{
  std::uint32_t trigger_source;
  std::uint32_t trigger_mode;
  std::uint32_t period;
  std::uint32_t offset;
  std::uint32_t duty_time;
  std::uint32_t threshold;
  std::uint32_t adjust_step;
};

struct LpwmAttributes
{
  std::uint32_t enable;
  std::array<LpwmChannelAttributes, kChannelCount> channels;
};

struct LpwmConfiguration
{
  std::uint32_t channel_id;
  LpwmAttributes attributes;
};

static_assert(sizeof(LpwmChannelAttributes) == 28U);
static_assert(sizeof(LpwmAttributes) == 116U);
static_assert(sizeof(LpwmConfiguration) == 120U);

constexpr unsigned long kLpwmInit = _IOW('L', 0x12, LpwmConfiguration);
constexpr unsigned long kLpwmClose = _IOW('L', 0x13, std::uint32_t);
static_assert(kLpwmInit == 0x40784c12UL);
static_assert(kLpwmClose == 0x40044c13UL);

struct Options
{
  std::string device{"/dev/hobot-lpwm1"};
  std::string pps_device{"/dev/pps-rtk"};
  std::string ready_file{"/run/agribot-camera-trigger/ready"};
  std::uint32_t channel_id{4U};
  std::uint32_t trigger_source{6U};
  std::uint32_t period_us{100000U};
  std::uint32_t offset_us{10U};
  std::uint32_t duty_us{1000U};
  std::uint32_t threshold_us{0U};
  std::uint32_t adjust_step{0U};
  double pps_timeout_sec{2.5};
};

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
  stop_requested = 1;
}

std::uint32_t parse_unsigned(
  const std::string & name, const std::string & value,
  const std::uint32_t minimum, const std::uint32_t maximum)
{
  std::size_t consumed = 0;
  const unsigned long parsed = std::stoul(value, &consumed, 0);
  if (consumed != value.size() || parsed < minimum || parsed > maximum) {
    throw std::invalid_argument(
            name + "必须在[" + std::to_string(minimum) + ", " +
            std::to_string(maximum) + "]范围内");
  }
  return static_cast<std::uint32_t>(parsed);
}

double parse_positive_double(const std::string & name, const std::string & value)
{
  std::size_t consumed = 0;
  const double parsed = std::stod(value, &consumed);
  if (consumed != value.size() || !std::isfinite(parsed) || parsed <= 0.0) {
    throw std::invalid_argument(name + "必须是正数");
  }
  return parsed;
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout <<
        "用法: camera_trigger_lpwm [--device PATH] [--pps-device PATH] "
        "[--ready-file PATH] [--channel-id N] [--trigger-source N] "
        "[--period-us N] [--offset-us N] [--duty-us N] "
        "[--threshold-us N] [--adjust-step N] [--pps-timeout-sec SEC]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("参数缺少值: " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--device") {
      options.device = value;
    } else if (argument == "--pps-device") {
      options.pps_device = value;
    } else if (argument == "--ready-file") {
      options.ready_file = value;
    } else if (argument == "--channel-id") {
      options.channel_id = parse_unsigned(argument, value, 0U, 15U);
    } else if (argument == "--trigger-source") {
      options.trigger_source = parse_unsigned(argument, value, 0U, 10U);
    } else if (argument == "--period-us") {
      options.period_us = parse_unsigned(argument, value, 2U, 1000000U);
    } else if (argument == "--offset-us") {
      options.offset_us = parse_unsigned(argument, value, 0U, 999999U);
    } else if (argument == "--duty-us") {
      options.duty_us = parse_unsigned(argument, value, 1U, 4000U);
    } else if (argument == "--threshold-us") {
      options.threshold_us = parse_unsigned(argument, value, 0U, 65535U);
    } else if (argument == "--adjust-step") {
      options.adjust_step = parse_unsigned(argument, value, 0U, 15U);
    } else if (argument == "--pps-timeout-sec") {
      options.pps_timeout_sec = parse_positive_double(argument, value);
    } else {
      throw std::invalid_argument("未知参数: " + argument);
    }
  }

  if (options.duty_us >= options.period_us) {
    throw std::invalid_argument("duty-us必须小于period-us");
  }
  if (options.offset_us + options.duty_us > options.period_us) {
    throw std::invalid_argument("offset-us与duty-us之和不能超过period-us");
  }
  return options;
}

class FileDescriptor
{
public:
  FileDescriptor(const std::string & path, const int flags)
  : value_(open(path.c_str(), flags | O_CLOEXEC))
  {
    if (value_ < 0) {
      throw std::runtime_error("无法打开" + path + ": " + std::strerror(errno));
    }
  }

  ~FileDescriptor()
  {
    if (value_ >= 0) {
      close(value_);
    }
  }

  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;

  int get() const {return value_;}

private:
  int value_;
};

pps_fdata fetch_pps(const int file_descriptor, const double timeout_sec)
{
  pps_fdata data{};
  data.timeout.sec = static_cast<std::int64_t>(timeout_sec);
  data.timeout.nsec = static_cast<std::int32_t>(
    (timeout_sec - static_cast<double>(data.timeout.sec)) * 1.0e9);
  data.timeout.flags = ~PPS_TIME_INVALID;
  if (ioctl(file_descriptor, PPS_FETCH, &data) != 0) {
    if (errno == EINTR && stop_requested != 0) {
      return data;
    }
    throw std::runtime_error("等待RTK PPS失败: " + std::string(std::strerror(errno)));
  }
  return data;
}

timespec realtime_now()
{
  timespec value{};
  if (clock_gettime(CLOCK_REALTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime失败: " + std::string(std::strerror(errno)));
  }
  return value;
}

double pps_age_seconds(const pps_fdata & event)
{
  const timespec now = realtime_now();
  const double now_sec = static_cast<double>(now.tv_sec) +
    static_cast<double>(now.tv_nsec) * 1.0e-9;
  const double event_sec = static_cast<double>(event.info.assert_tu.sec) +
    static_cast<double>(event.info.assert_tu.nsec) * 1.0e-9;
  return now_sec - event_sec;
}

void require_fresh_pps(const pps_fdata & event, const double timeout_sec)
{
  const double age = pps_age_seconds(event);
  if (event.info.assert_sequence == 0U || age < 0.0 || age > timeout_sec) {
    throw std::runtime_error("RTK PPS事件无效或已经过期");
  }
}

LpwmConfiguration make_configuration(const Options & options)
{
  LpwmConfiguration configuration{};
  configuration.channel_id = options.channel_id;
  configuration.attributes.enable = 1U;
  auto & channel = configuration.attributes.channels.at(options.channel_id % kChannelCount);
  channel.trigger_source = options.trigger_source;
  channel.trigger_mode = kHardwareTriggerMode;
  channel.period = options.period_us - 1U;
  channel.offset = options.offset_us;
  channel.duty_time = options.duty_us - 1U;
  channel.threshold = options.threshold_us;
  channel.adjust_step = options.adjust_step;
  return configuration;
}

void write_ready_file(const Options & options, const pps_fdata & event)
{
  const std::string temporary = options.ready_file + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("无法创建LPWM就绪文件: " + temporary);
  }
  output << "backend=j14_lpwm\n"
         << "device=" << options.device << '\n'
         << "channel_id=" << options.channel_id << '\n'
         << "channel=" << options.channel_id % kChannelCount << '\n'
         << "trigger_source=" << options.trigger_source << '\n'
         << "trigger_mode=" << kHardwareTriggerMode << '\n'
         << "period_us=" << options.period_us << '\n'
         << "offset_us=" << options.offset_us << '\n'
         << "duty_us=" << options.duty_us << '\n'
         << "threshold_us=" << options.threshold_us << '\n'
         << "adjust_step=" << options.adjust_step << '\n'
         << "pps_sequence=" << event.info.assert_sequence << '\n'
         << "pid=" << getpid() << '\n';
  output.close();
  if (!output) {
    unlink(temporary.c_str());
    throw std::runtime_error("写入LPWM就绪文件失败: " + temporary);
  }
  if (rename(temporary.c_str(), options.ready_file.c_str()) != 0) {
    const std::string error = std::strerror(errno);
    unlink(temporary.c_str());
    throw std::runtime_error("发布LPWM就绪文件失败: " + error);
  }
}

class ReadyFileGuard
{
public:
  explicit ReadyFileGuard(std::string path) : path_(std::move(path)) {}
  ~ReadyFileGuard() {unlink(path_.c_str());}

private:
  std::string path_;
};

void install_signal_handlers()
{
  struct sigaction action {};
  action.sa_handler = signal_handler;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(SIGTERM, &action, nullptr) != 0 ||
    sigaction(SIGINT, &action, nullptr) != 0)
  {
    throw std::runtime_error(
            "安装退出信号处理失败: " + std::string(std::strerror(errno)));
  }
}

void run(const Options & options)
{
  install_signal_handlers();
  FileDescriptor pps(options.pps_device, O_RDONLY);
  FileDescriptor lpwm(options.device, O_RDWR);

  const pps_fdata initial_event = fetch_pps(pps.get(), options.pps_timeout_sec);
  require_fresh_pps(initial_event, options.pps_timeout_sec);

  LpwmConfiguration configuration = make_configuration(options);
  if (ioctl(lpwm.get(), kLpwmInit, &configuration) != 0) {
    throw std::runtime_error(
            "配置LPWM硬件PPS触发失败: " + std::string(std::strerror(errno)));
  }

  bool configured = true;
  ReadyFileGuard ready_guard(options.ready_file);
  try {
    write_ready_file(options, initial_event);
    std::cout << "LPWM硬件触发已启动: device=" << options.device
              << ", channel_id=" << options.channel_id
              << ", source=" << options.trigger_source
              << ", period=" << options.period_us << " us"
              << ", offset=" << options.offset_us << " us"
              << ", duty=" << options.duty_us << " us"
              << ", initial_pps=" << initial_event.info.assert_sequence << std::endl;

    std::uint32_t last_sequence = initial_event.info.assert_sequence;
    while (stop_requested == 0) {
      const pps_fdata event = fetch_pps(pps.get(), options.pps_timeout_sec);
      if (stop_requested != 0) {
        break;
      }
      require_fresh_pps(event, options.pps_timeout_sec);
      if (event.info.assert_sequence == last_sequence) {
        throw std::runtime_error("RTK PPS序号在超时时间内没有递增");
      }
      last_sequence = event.info.assert_sequence;
    }
  } catch (...) {
    if (configured) {
      ioctl(lpwm.get(), kLpwmClose, &configuration.channel_id);
      configured = false;
    }
    throw;
  }

  if (configured && ioctl(lpwm.get(), kLpwmClose, &configuration.channel_id) != 0) {
    throw std::runtime_error("停止LPWM失败: " + std::string(std::strerror(errno)));
  }
  std::cout << "LPWM相机触发已停止" << std::endl;
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    run(parse_options(argc, argv));
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
  }
}
