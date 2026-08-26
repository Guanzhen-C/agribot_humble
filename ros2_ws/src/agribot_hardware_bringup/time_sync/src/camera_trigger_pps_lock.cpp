#include <fcntl.h>
#include <linux/pps.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace
{
struct Options
{
  std::string pps_device{"/dev/pps-rtk"};
  std::string pwm_enable_path{"/sys/class/pwm/pwmchip0/pwm0/enable"};
  std::string ready_file{"/run/agribot-camera-trigger/ready"};
  std::uint64_t period_ns{100000000U};
  std::uint64_t duty_cycle_ns{1000000U};
  std::string polarity{"normal"};
  double timeout_sec{2.5};
  double maximum_latency_ms{5.0};
  double guard_ms{10.0};
};

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int)
{
  stop_requested = 1;
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

std::uint64_t parse_positive_unsigned(const std::string & name, const std::string & value)
{
  std::size_t consumed = 0;
  const unsigned long long parsed = std::stoull(value, &consumed, 0);
  if (consumed != value.size() || parsed == 0U) {
    throw std::invalid_argument(name + "必须是正整数");
  }
  return static_cast<std::uint64_t>(parsed);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout <<
        "用法: camera_trigger_pps_lock [--pps-device PATH] "
        "[--pwm-enable-path PATH] [--ready-file PATH] "
        "[--period-ns N] [--duty-cycle-ns N] [--polarity VALUE] "
        "[--timeout-sec SEC] [--maximum-latency-ms MS] [--guard-ms MS]\n";
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
    } else if (argument == "--ready-file") {
      options.ready_file = value;
    } else if (argument == "--period-ns") {
      options.period_ns = parse_positive_unsigned(argument, value);
    } else if (argument == "--duty-cycle-ns") {
      options.duty_cycle_ns = parse_positive_unsigned(argument, value);
    } else if (argument == "--polarity") {
      options.polarity = value;
    } else if (argument == "--timeout-sec") {
      options.timeout_sec = parse_positive_double(argument, value);
    } else if (argument == "--maximum-latency-ms") {
      options.maximum_latency_ms = parse_positive_double(argument, value);
    } else if (argument == "--guard-ms") {
      options.guard_ms = parse_positive_double(argument, value);
    } else {
      throw std::invalid_argument("未知参数: " + argument);
    }
  }
  if (options.duty_cycle_ns >= options.period_ns) {
    throw std::invalid_argument("duty-cycle-ns必须小于period-ns");
  }
  if (options.polarity != "normal" && options.polarity != "inversed") {
    throw std::invalid_argument("polarity必须是normal或inversed");
  }
  if (options.guard_ms >= 1000.0) {
    throw std::invalid_argument("guard-ms必须小于1000");
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

void write_pwm_enable(const int file_descriptor, const bool enabled)
{
  if (lseek(file_descriptor, 0, SEEK_SET) < 0) {
    throw std::runtime_error("重置PWM使能文件偏移失败: " + std::string(std::strerror(errno)));
  }
  const char * value = enabled ? "1\n" : "0\n";
  if (write(file_descriptor, value, 2) != 2) {
    throw std::runtime_error("写入PWM使能状态失败: " + std::string(std::strerror(errno)));
  }
}

timespec realtime_now()
{
  timespec value{};
  if (clock_gettime(CLOCK_REALTIME, &value) != 0) {
    throw std::runtime_error("clock_gettime失败: " + std::string(std::strerror(errno)));
  }
  return value;
}

double timespec_seconds(const timespec & value)
{
  return static_cast<double>(value.tv_sec) + static_cast<double>(value.tv_nsec) * 1.0e-9;
}

timespec next_guard_time(const pps_fdata & event, const double guard_ms)
{
  std::int64_t nanoseconds = static_cast<std::int64_t>(event.info.assert_tu.nsec) +
    1000000000LL - static_cast<std::int64_t>(std::llround(guard_ms * 1.0e6));
  timespec target{};
  target.tv_sec = static_cast<time_t>(event.info.assert_tu.sec);
  while (nanoseconds >= 1000000000LL) {
    ++target.tv_sec;
    nanoseconds -= 1000000000LL;
  }
  while (nanoseconds < 0) {
    --target.tv_sec;
    nanoseconds += 1000000000LL;
  }
  target.tv_nsec = static_cast<long>(nanoseconds);
  return target;
}

void sleep_until(const timespec & target)
{
  while (stop_requested == 0) {
    const int result = clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &target, nullptr);
    if (result == 0) {
      return;
    }
    if (result != EINTR) {
      throw std::runtime_error("等待下一次PPS保护窗口失败: " + std::string(std::strerror(result)));
    }
  }
}

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

double event_age_ms(const pps_fdata & event, const timespec & now)
{
  const double event_seconds = static_cast<double>(event.info.assert_tu.sec) +
    static_cast<double>(event.info.assert_tu.nsec) * 1.0e-9;
  return (timespec_seconds(now) - event_seconds) * 1.0e3;
}

double require_fresh_pps(
  const pps_fdata & event, const std::uint32_t previous_sequence,
  const double maximum_latency_ms)
{
  const timespec now = realtime_now();
  const double age_ms = event_age_ms(event, now);
  if (event.info.assert_sequence == 0U || event.info.assert_sequence == previous_sequence) {
    throw std::runtime_error("RTK PPS序号在超时时间内没有递增");
  }
  if (age_ms < 0.0 || age_ms > maximum_latency_ms) {
    throw std::runtime_error(
            "PPS到PWM校相延迟超限: " + std::to_string(age_ms) + " ms");
  }
  return age_ms;
}

void request_realtime_scheduling()
{
  sched_param parameters{};
  parameters.sched_priority = 20;
  if (sched_setscheduler(0, SCHED_FIFO, &parameters) != 0) {
    std::cerr << "警告: 无法启用SCHED_FIFO，继续使用普通调度: "
              << std::strerror(errno) << '\n';
  }
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "警告: 无法锁定内存，继续运行: " << std::strerror(errno) << '\n';
  }
}

void write_ready_file(
  const Options & options, const pps_fdata & event, const double latency_ms)
{
  const std::string temporary = options.ready_file + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("无法创建PWM就绪文件: " + temporary);
  }
  output << "backend=pin32_pwm\n"
         << "pwm_enable_path=" << options.pwm_enable_path << '\n'
         << "period_ns=" << options.period_ns << '\n'
         << "duty_cycle_ns=" << options.duty_cycle_ns << '\n'
         << "polarity=" << options.polarity << '\n'
         << "pps_rephase=continuous\n"
         << "pps_sequence=" << event.info.assert_sequence << '\n'
         << "pps_sec=" << event.info.assert_tu.sec << '\n'
         << "pps_nsec=" << event.info.assert_tu.nsec << '\n'
         << std::fixed << std::setprecision(3)
         << "last_latency_us=" << latency_ms * 1.0e3 << '\n'
         << "guard_ms=" << options.guard_ms << '\n'
         << "pid=" << getpid() << '\n';
  output.close();
  if (!output) {
    unlink(temporary.c_str());
    throw std::runtime_error("写入PWM就绪文件失败: " + temporary);
  }
  if (rename(temporary.c_str(), options.ready_file.c_str()) != 0) {
    const std::string error = std::strerror(errno);
    unlink(temporary.c_str());
    throw std::runtime_error("发布PWM就绪文件失败: " + error);
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
    throw std::runtime_error("安装退出信号处理失败: " + std::string(std::strerror(errno)));
  }
}

void run(const Options & options)
{
  install_signal_handlers();
  if (read_trimmed(options.pwm_enable_path) != "0") {
    throw std::runtime_error("Pin32 PWM必须在校相程序启动前保持关闭");
  }

  FileDescriptor pps(options.pps_device, O_RDONLY);
  FileDescriptor pwm(options.pwm_enable_path, O_WRONLY);
  ReadyFileGuard ready_guard(options.ready_file);
  request_realtime_scheduling();

  bool enabled = false;
  try {
    pps_fdata event = fetch_pps(pps.get(), options.timeout_sec);
    double latency_ms = require_fresh_pps(event, 0U, options.maximum_latency_ms);
    write_pwm_enable(pwm.get(), true);
    enabled = true;
    latency_ms = event_age_ms(event, realtime_now());
    write_ready_file(options, event, latency_ms);
    std::cout << "Pin32相机触发已启动: 10 Hz, initial_pps="
              << event.info.assert_sequence << ", latency="
              << latency_ms * 1.0e3 << " us" << std::endl;

    std::uint32_t last_sequence = event.info.assert_sequence;
    while (stop_requested == 0) {
      sleep_until(next_guard_time(event, options.guard_ms));
      if (stop_requested != 0) {
        break;
      }
      write_pwm_enable(pwm.get(), false);
      enabled = false;

      event = fetch_pps(pps.get(), options.timeout_sec);
      if (stop_requested != 0) {
        break;
      }
      latency_ms = require_fresh_pps(event, last_sequence, options.maximum_latency_ms);
      write_pwm_enable(pwm.get(), true);
      enabled = true;
      latency_ms = event_age_ms(event, realtime_now());
      if (latency_ms > options.maximum_latency_ms) {
        throw std::runtime_error(
                "使能Pin32 PWM后的校相延迟超限: " + std::to_string(latency_ms) + " ms");
      }
      last_sequence = event.info.assert_sequence;
      write_ready_file(options, event, latency_ms);
    }
  } catch (...) {
    if (enabled) {
      try {
        write_pwm_enable(pwm.get(), false);
      } catch (const std::exception & error) {
        std::cerr << "警告: 异常退出时关闭Pin32 PWM失败: " << error.what() << '\n';
      }
    }
    throw;
  }

  if (enabled) {
    write_pwm_enable(pwm.get(), false);
  }
  std::cout << "Pin32相机触发已停止" << std::endl;
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
