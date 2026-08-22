#include <fcntl.h>
#include <linux/pps.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
struct Options
{
  std::string pps_device{"/dev/pps-rtk"};
  std::string pwm_enable_path{"/sys/class/pwm/pwmchip0/pwm0/enable"};
  double timeout_sec{5.0};
  double maximum_latency_ms{5.0};
};

double timespec_seconds(const timespec & value)
{
  return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1.0e-9;
}

timespec realtime_now()
{
  timespec value{};
  if (clock_gettime(CLOCK_REALTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime失败: " + std::string(std::strerror(errno)));
  }
  return value;
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
      std::cout << "用法: camera_trigger_pps_lock [--pps-device PATH] "
                   "[--pwm-enable-path PATH] [--timeout-sec SEC] "
                   "[--maximum-latency-ms MS]\n";
      std::exit(0);
    }
    if (index + 1 >= argc) {
      throw std::invalid_argument("参数缺少值: " + argument);
    }
    const std::string value(argv[++index]);
    if (argument == "--pps-device") {
      options.pps_device = value;
    } else if (argument == "--pwm-enable-path") {
      options.pwm_enable_path = value;
    } else if (argument == "--timeout-sec") {
      options.timeout_sec = parse_positive_double(argument, value);
    } else if (argument == "--maximum-latency-ms") {
      options.maximum_latency_ms = parse_positive_double(argument, value);
    } else {
      throw std::invalid_argument("未知参数: " + argument);
    }
  }
  return options;
}

std::string read_trimmed(const std::string & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("无法读取" + path);
  }
  std::string value;
  input >> value;
  return value;
}

void request_realtime_scheduling()
{
  sched_param parameters{};
  parameters.sched_priority = 20;
  if (sched_setscheduler(0, SCHED_FIFO, &parameters) != 0) {
    std::cerr << "警告: 无法启用SCHED_FIFO，继续使用普通调度: "
              << std::strerror(errno) << '\n';
  }
}

pps_fdata fetch_pps(int file_descriptor, double timeout_sec)
{
  pps_fdata data{};
  data.timeout.sec = static_cast<std::int64_t>(timeout_sec);
  data.timeout.nsec = static_cast<std::int32_t>(
    (timeout_sec - static_cast<double>(data.timeout.sec)) * 1.0e9);
  data.timeout.flags = ~PPS_TIME_INVALID;
  if (ioctl(file_descriptor, PPS_FETCH, &data) != 0) {
    throw std::runtime_error("等待PPS失败: " + std::string(std::strerror(errno)));
  }
  return data;
}

void enable_pwm_at_next_pps(const Options & options)
{
  if (read_trimmed(options.pwm_enable_path) != "0") {
    throw std::runtime_error("PWM必须先保持关闭: " + options.pwm_enable_path);
  }

  const int pps_fd = open(options.pps_device.c_str(), O_RDONLY | O_CLOEXEC);
  if (pps_fd < 0) {
    throw std::runtime_error(
            "无法打开PPS设备" + options.pps_device + ": " + std::strerror(errno));
  }
  const int pwm_fd = open(options.pwm_enable_path.c_str(), O_WRONLY | O_CLOEXEC);
  if (pwm_fd < 0) {
    const std::string error = std::strerror(errno);
    close(pps_fd);
    throw std::runtime_error("无法打开PWM使能文件: " + error);
  }

  request_realtime_scheduling();
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(options.timeout_sec);
  pps_fdata event{};
  double age_ms = options.maximum_latency_ms + 1.0;
  while (std::chrono::steady_clock::now() < deadline) {
    const double remaining = std::chrono::duration<double>(
      deadline - std::chrono::steady_clock::now()).count();
    event = fetch_pps(pps_fd, std::max(remaining, 0.001));
    const timespec now = realtime_now();
    const double event_sec = static_cast<double>(event.info.assert_tu.sec) +
      static_cast<double>(event.info.assert_tu.nsec) * 1.0e-9;
    age_ms = (timespec_seconds(now) - event_sec) * 1.0e3;
    if (age_ms >= 0.0 && age_ms <= options.maximum_latency_ms) {
      break;
    }
  }

  if (age_ms < 0.0 || age_ms > options.maximum_latency_ms) {
    close(pwm_fd);
    close(pps_fd);
    throw std::runtime_error("超时前未获得足够新鲜的PPS事件");
  }

  if (write(pwm_fd, "1\n", 2) != 2) {
    const std::string error = std::strerror(errno);
    close(pwm_fd);
    close(pps_fd);
    throw std::runtime_error("使能PWM失败: " + error);
  }
  const timespec enabled_at = realtime_now();
  const double event_sec = static_cast<double>(event.info.assert_tu.sec) +
    static_cast<double>(event.info.assert_tu.nsec) * 1.0e-9;
  const double enable_latency_us = (timespec_seconds(enabled_at) - event_sec) * 1.0e6;
  close(pwm_fd);
  close(pps_fd);

  std::cout << "PPS序号: " << event.info.assert_sequence << '\n'
            << "PPS时刻: " << event.info.assert_tu.sec << '.'
            << event.info.assert_tu.nsec << '\n'
            << "PWM使能延迟: " << enable_latency_us << " us\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    enable_pwm_at_next_pps(parse_options(argc, argv));
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "错误: " << error.what() << '\n';
    return 1;
  }
}
