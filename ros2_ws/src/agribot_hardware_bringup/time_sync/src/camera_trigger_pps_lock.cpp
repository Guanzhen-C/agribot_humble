#include <fcntl.h>
#include <linux/gpio.h>
#include <linux/pps.h>
#include <poll.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <agribot_time_sync/trigger_edge_ring.hpp>

namespace
{
struct Options
{
  std::string pps_device{"/dev/pps-rtk"};
  std::string pwm_enable_path{"/sys/class/pwm/pwmchip0/pwm0/enable"};
  std::string pwm_period_path{"/sys/class/pwm/pwmchip0/pwm0/period"};
  std::string ready_file{"/run/agribot-camera-trigger/ready"};
  std::string edge_gpio_chip{"/dev/gpiochip5"};
  std::uint32_t edge_gpio_offset{10U};
  std::string edge_buffer_path{"/run/agribot-camera-trigger/physical_edges.bin"};
  std::uint64_t period_ns{100000000U};
  std::uint64_t duty_cycle_ns{1000000U};
  std::string polarity{"normal"};
  double timeout_sec{2.5};
  double maximum_latency_ms{5.0};
  double period_update_compensation_ms{1.0};
  std::uint64_t maximum_period_adjustment_ns{500000U};
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

std::uint32_t parse_unsigned(const std::string & name, const std::string & value)
{
  std::size_t consumed = 0;
  const unsigned long parsed = std::stoul(value, &consumed, 0);
  if (consumed != value.size() || parsed > UINT32_MAX) {
    throw std::invalid_argument(name + "必须是32位非负整数");
  }
  return static_cast<std::uint32_t>(parsed);
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout <<
        "用法: camera_trigger_pps_lock [--pps-device PATH] "
        "[--pwm-enable-path PATH] [--pwm-period-path PATH] [--ready-file PATH] "
        "[--edge-gpio-chip PATH] [--edge-gpio-offset N] [--edge-buffer-path PATH] "
        "[--period-ns N] [--duty-cycle-ns N] [--polarity VALUE] "
        "[--timeout-sec SEC] [--maximum-latency-ms MS] "
        "[--period-update-compensation-ms MS] "
        "[--maximum-period-adjustment-ns N]\n";
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
    } else if (argument == "--pwm-period-path") {
      options.pwm_period_path = value;
    } else if (argument == "--ready-file") {
      options.ready_file = value;
    } else if (argument == "--edge-gpio-chip") {
      options.edge_gpio_chip = value;
    } else if (argument == "--edge-gpio-offset") {
      options.edge_gpio_offset = parse_unsigned(argument, value);
    } else if (argument == "--edge-buffer-path") {
      options.edge_buffer_path = value;
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
    } else if (argument == "--period-update-compensation-ms") {
      options.period_update_compensation_ms = parse_positive_double(argument, value);
    } else if (argument == "--maximum-period-adjustment-ns") {
      options.maximum_period_adjustment_ns = parse_positive_unsigned(argument, value);
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
  if (1000000000ULL % options.period_ns != 0U) {
    throw std::invalid_argument("period-ns必须能整除1秒，才能逐PPS无丢帧校相");
  }
  if (options.period_update_compensation_ms >= options.maximum_latency_ms) {
    throw std::invalid_argument(
            "period-update-compensation-ms必须小于maximum-latency-ms");
  }
  if (options.maximum_period_adjustment_ns * 2U >= options.period_ns) {
    throw std::invalid_argument("maximum-period-adjustment-ns必须小于半个PWM周期");
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

void write_pwm_period(const int file_descriptor, const std::uint64_t period_ns)
{
  if (lseek(file_descriptor, 0, SEEK_SET) < 0) {
    throw std::runtime_error("重置PWM周期文件偏移失败: " + std::string(std::strerror(errno)));
  }
  const std::string value = std::to_string(period_ns) + "\n";
  if (write(file_descriptor, value.data(), value.size()) != static_cast<ssize_t>(value.size())) {
    throw std::runtime_error("写入PWM周期失败: " + std::string(std::strerror(errno)));
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

bool fetch_pps(
  const int file_descriptor, const double timeout_sec, pps_fdata & data)
{
  data = {};
  data.timeout.sec = static_cast<std::int64_t>(timeout_sec);
  data.timeout.nsec = static_cast<std::int32_t>(
    (timeout_sec - static_cast<double>(data.timeout.sec)) * 1.0e9);
  data.timeout.flags = ~PPS_TIME_INVALID;
  if (ioctl(file_descriptor, PPS_FETCH, &data) != 0) {
    if (errno == ETIMEDOUT) {
      return false;
    }
    if (errno == EINTR && stop_requested != 0) {
      return false;
    }
    throw std::runtime_error("等待RTK PPS失败: " + std::string(std::strerror(errno)));
  }
  return true;
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

struct EdgeSnapshot
{
  std::uint64_t count{0U};
  std::uint64_t raw_count{0U};
  std::uint64_t rejected_count{0U};
  std::uint64_t ring_sequence{0U};
  std::uint64_t ring_generation{0U};
  std::int64_t timestamp_ns{0};
  std::uint32_t kernel_sequence{0U};
};

class GpioEdgeCapture
{
public:
  explicit GpioEdgeCapture(const Options & options)
  : ring_(options.edge_buffer_path), minimum_interval_ns_(options.period_ns / 2U)
  {
    chip_descriptor_ = open(options.edge_gpio_chip.c_str(), O_RDONLY | O_CLOEXEC);
    if (chip_descriptor_ < 0) {
      throw std::runtime_error(
              "无法打开Pin33 GPIO控制器" + options.edge_gpio_chip + ": " +
              std::strerror(errno));
    }

    gpio_v2_line_request request{};
    request.offsets[0] = options.edge_gpio_offset;
    request.num_lines = 1U;
    request.event_buffer_size = 64U;
    std::strncpy(request.consumer, "agribot-camera-edge", sizeof(request.consumer) - 1U);
    request.config.flags = GPIO_V2_LINE_FLAG_INPUT |
      GPIO_V2_LINE_FLAG_EDGE_RISING | GPIO_V2_LINE_FLAG_EVENT_CLOCK_REALTIME;
    if (ioctl(chip_descriptor_, GPIO_V2_GET_LINE_IOCTL, &request) != 0) {
      const std::string error = std::strerror(errno);
      close(chip_descriptor_);
      chip_descriptor_ = -1;
      throw std::runtime_error("无法申请Pin33上升沿中断: " + error);
    }
    line_descriptor_ = request.fd;
    running_.store(true);
    thread_ = std::thread(&GpioEdgeCapture::capture_loop, this);
  }

  ~GpioEdgeCapture()
  {
    running_.store(false);
    if (thread_.joinable()) {
      thread_.join();
    }
    if (line_descriptor_ >= 0) {
      close(line_descriptor_);
    }
    if (chip_descriptor_ >= 0) {
      close(chip_descriptor_);
    }
  }

  GpioEdgeCapture(const GpioEdgeCapture &) = delete;
  GpioEdgeCapture & operator=(const GpioEdgeCapture &) = delete;

  EdgeSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    require_healthy_locked();
    return snapshot_;
  }

  EdgeSnapshot wait_after(const std::uint64_t count, const double timeout_ms)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const bool ready = condition_.wait_for(
      lock, std::chrono::duration<double, std::milli>(timeout_ms),
      [this, count]() {return snapshot_.count > count || !error_.empty();});
    require_healthy_locked();
    if (!ready || snapshot_.count <= count) {
      throw std::runtime_error("Pin33未在时限内捕获到PWM物理上升沿");
    }
    return snapshot_;
  }

private:
  void require_healthy_locked() const
  {
    if (!error_.empty()) {
      throw std::runtime_error("Pin33物理沿采集失败: " + error_);
    }
  }

  void capture_loop()
  {
    while (running_.load() && stop_requested == 0) {
      pollfd descriptor{};
      descriptor.fd = line_descriptor_;
      descriptor.events = POLLIN;
      const int result = poll(&descriptor, 1, 100);
      if (result == 0 || (result < 0 && errno == EINTR)) {
        continue;
      }
      if (result < 0) {
        publish_error(std::strerror(errno));
        return;
      }
      if ((descriptor.revents & POLLIN) == 0) {
        publish_error("GPIO事件文件返回异常状态");
        return;
      }
      gpio_v2_line_event event{};
      const ssize_t size = read(line_descriptor_, &event, sizeof(event));
      if (size != static_cast<ssize_t>(sizeof(event))) {
        if (size < 0 && (errno == EINTR || errno == EAGAIN)) {
          continue;
        }
        publish_error(size < 0 ? std::strerror(errno) : "GPIO事件长度错误");
        return;
      }
      if (event.id != GPIO_V2_LINE_EVENT_RISING_EDGE) {
        continue;
      }
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.raw_count;
        const std::int64_t timestamp_ns = static_cast<std::int64_t>(event.timestamp_ns);
        if (snapshot_.timestamp_ns > 0 &&
          timestamp_ns - snapshot_.timestamp_ns < static_cast<std::int64_t>(minimum_interval_ns_))
        {
          ++snapshot_.rejected_count;
          continue;
        }
      }
      const std::uint64_t ring_sequence = ring_.write(
        static_cast<std::int64_t>(event.timestamp_ns), event.line_seqno);
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.count;
        snapshot_.ring_sequence = ring_sequence;
        snapshot_.ring_generation = ring_.generation();
        snapshot_.timestamp_ns = static_cast<std::int64_t>(event.timestamp_ns);
        snapshot_.kernel_sequence = event.line_seqno;
      }
      condition_.notify_all();
    }
  }

  void publish_error(const std::string & error)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      error_ = error;
    }
    condition_.notify_all();
  }

  agribot_time_sync::TriggerEdgeRingWriter ring_;
  std::uint64_t minimum_interval_ns_{0U};
  int chip_descriptor_{-1};
  int line_descriptor_{-1};
  std::atomic<bool> running_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::condition_variable condition_;
  EdgeSnapshot snapshot_;
  std::string error_;
};

std::int64_t pps_timestamp_ns(const pps_fdata & event)
{
  return static_cast<std::int64_t>(event.info.assert_tu.sec) * 1000000000LL +
    static_cast<std::int64_t>(event.info.assert_tu.nsec);
}

double physical_phase_ms(const EdgeSnapshot & edge, const pps_fdata & event)
{
  return static_cast<double>(edge.timestamp_ns - pps_timestamp_ns(event)) * 1.0e-6;
}

EdgeSnapshot match_pps_edge(
  GpioEdgeCapture & capture, const pps_fdata & event,
  const std::uint64_t previous_pps_edge_count, const std::uint64_t expected_edges,
  const double maximum_latency_ms)
{
  EdgeSnapshot edge = capture.snapshot();
  double phase_ms = physical_phase_ms(edge, event);
  if (edge.count <= previous_pps_edge_count || std::abs(phase_ms) > maximum_latency_ms) {
    const std::uint64_t count = edge.count;
    edge = capture.wait_after(count, maximum_latency_ms + 2.0);
    phase_ms = physical_phase_ms(edge, event);
  }
  if (edge.count - previous_pps_edge_count != expected_edges) {
    throw std::runtime_error(
            "相邻PPS之间的Pin33物理沿数错误: 实际" +
            std::to_string(edge.count - previous_pps_edge_count) + "，期望" +
            std::to_string(expected_edges));
  }
  if (std::abs(phase_ms) > maximum_latency_ms) {
    throw std::runtime_error(
            "Pin33物理上升沿相对PPS的相位超限: " + std::to_string(phase_ms) + " ms");
  }
  return edge;
}

std::uint64_t calculate_locked_period(
  const Options & options, const EdgeSnapshot & edge, const pps_fdata & event)
{
  const std::int64_t compensation_ns =
    static_cast<std::int64_t>(options.period_update_compensation_ms * 1.0e6);
  const std::int64_t phase_ns = edge.timestamp_ns - pps_timestamp_ns(event);
  const timespec now = realtime_now();
  const std::int64_t now_ns = static_cast<std::int64_t>(now.tv_sec) * 1000000000LL + now.tv_nsec;
  const std::int64_t dispatch_ns = std::max<std::int64_t>(now_ns - edge.timestamp_ns, 0);
  const std::int64_t pulses = static_cast<std::int64_t>(1000000000ULL / options.period_ns);
  std::int64_t adjustment = (compensation_ns - phase_ns - dispatch_ns) / pulses;
  const std::int64_t limit = static_cast<std::int64_t>(options.maximum_period_adjustment_ns);
  adjustment = std::clamp(adjustment, -limit, limit);
  return static_cast<std::uint64_t>(static_cast<std::int64_t>(options.period_ns) + adjustment);
}

void write_ready_file(
  const Options & options, const pps_fdata & event,
  const std::uint32_t initial_sequence, const double initial_edge_phase_ms,
  const double last_pps_latency_ms, const double edge_phase_ms,
  const std::uint64_t edges_previous_second, const std::uint64_t applied_period_ns,
  const EdgeSnapshot & edge, const bool pps_locked = true)
{
  const std::string temporary = options.ready_file + ".tmp";
  std::ofstream output(temporary, std::ios::trunc);
  if (!output) {
    throw std::runtime_error("无法创建PWM就绪文件: " + temporary);
  }
  output << "backend=pin32_pwm\n"
         << "pwm_enable_path=" << options.pwm_enable_path << '\n'
         << "pwm_period_path=" << options.pwm_period_path << '\n'
         << "period_ns=" << options.period_ns << '\n'
         << "duty_cycle_ns=" << options.duty_cycle_ns << '\n'
         << "polarity=" << options.polarity << '\n'
         << "pps_alignment=" << (pps_locked ? "every_pps" : "holdover") << '\n'
         << "pps_monitoring=continuous\n"
         << "pps_lock_state=" << (pps_locked ? "locked" : "holdover") << '\n'
         << "pwm_phase_control=" <<
    (pps_locked ? "period_adjust_each_pps" : "nominal_period") << '\n'
         << "pulses_per_pps=" << 1000000000ULL / options.period_ns << '\n'
         << "nominal_period_ns=" << options.period_ns << '\n'
         << "applied_period_ns=" << applied_period_ns << '\n'
         << "phase_lock_target_us=0\n"
         << "period_update_compensation_us=" <<
    options.period_update_compensation_ms * 1.0e3 << '\n'
         << "physical_edge_capture=pin33_gpio\n"
         << "edge_timestamp_source=gpio_v2_realtime\n"
         << "edge_gpio_chip=" << options.edge_gpio_chip << '\n'
         << "edge_gpio_offset=" << options.edge_gpio_offset << '\n'
         << "edge_buffer_path=" << options.edge_buffer_path << '\n'
         << "edge_buffer_generation=" << edge.ring_generation << '\n'
         << "edge_sequence=" << edge.ring_sequence << '\n'
         << "edge_kernel_sequence=" << edge.kernel_sequence << '\n'
         << "edge_timestamp_ns=" << edge.timestamp_ns << '\n'
         << "edge_count=" << edge.count << '\n'
         << "edge_raw_count=" << edge.raw_count << '\n'
         << "edge_rejected_count=" << edge.rejected_count << '\n'
         << "edge_filter=minimum_half_period\n"
         << "edges_previous_second=" << edges_previous_second << '\n'
         << "initial_pps_sequence=" << initial_sequence << '\n'
         << std::fixed << std::setprecision(3)
         << "initial_edge_phase_us=" << initial_edge_phase_ms * 1.0e3 << '\n'
         << "pps_sequence=" << event.info.assert_sequence << '\n'
         << "pps_sec=" << event.info.assert_tu.sec << '\n'
         << "pps_nsec=" << event.info.assert_tu.nsec << '\n'
         << "last_pps_latency_us=" << last_pps_latency_ms * 1.0e3 << '\n'
         << "edge_phase_error_us=" << edge_phase_ms * 1.0e3 << '\n'
         << "phase_lock_error_us=" << edge_phase_ms * 1.0e3 << '\n'
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
  FileDescriptor pwm_period(options.pwm_period_path, O_WRONLY);
  ReadyFileGuard ready_guard(options.ready_file);
  GpioEdgeCapture edge_capture(options);
  request_realtime_scheduling();

  bool enabled = false;
  try {
    pps_fdata event{};
    const bool initial_pps_available = fetch_pps(
      pps.get(), options.timeout_sec, event);
    if (stop_requested != 0) {
      return;
    }
    if (!initial_pps_available) {
      const std::uint64_t expected_edges = 1000000000ULL / options.period_ns;
      EdgeSnapshot edge = edge_capture.snapshot();
      const std::uint64_t initial_edge_count = edge.count;
      write_pwm_enable(pwm.get(), true);
      enabled = true;
      for (std::uint64_t index = 0; index < expected_edges; ++index) {
        edge = edge_capture.wait_after(
          edge.count, static_cast<double>(options.period_ns) * 2.0e-6);
      }
      if (edge.count - initial_edge_count != expected_edges) {
        throw std::runtime_error("PPS保持模式未检测到完整的10 Hz物理触发沿");
      }
      pps_fdata empty_event{};
      write_ready_file(
        options, empty_event, 0U, 0.0, 0.0, 0.0, expected_edges,
        options.period_ns, edge, false);
      std::cout << "RTK PPS暂不可用，Pin32相机触发进入10 Hz保持模式；"
                << "继续发布Pin33物理沿时间戳并等待PPS恢复" << std::endl;

      const double holdover_poll_sec = std::min(
        options.timeout_sec, static_cast<double>(options.period_ns) * 1.0e-9);
      while (stop_requested == 0) {
        pps_fdata restored_event{};
        if (fetch_pps(pps.get(), holdover_poll_sec, restored_event)) {
          throw std::runtime_error("检测到RTK PPS恢复，重启触发服务以重新进入逐PPS锁相");
        }
        if (stop_requested != 0) {
          break;
        }
        edge = edge_capture.snapshot();
        const double edge_age_ms =
          (timespec_seconds(realtime_now()) -
          static_cast<double>(edge.timestamp_ns) * 1.0e-9) * 1.0e3;
        if (edge_age_ms < 0.0 ||
          edge_age_ms > static_cast<double>(options.period_ns) * 2.0e-6)
        {
          throw std::runtime_error("PPS保持模式的Pin33物理触发沿已停止");
        }
        write_ready_file(
          options, empty_event, 0U, 0.0, 0.0, 0.0, expected_edges,
          options.period_ns, edge, false);
      }
    } else {
      require_fresh_pps(event, 0U, options.maximum_latency_ms);
      EdgeSnapshot before_enable = edge_capture.snapshot();
      write_pwm_enable(pwm.get(), true);
      enabled = true;
      const std::uint32_t initial_sequence = event.info.assert_sequence;
      const std::uint64_t expected_edges = 1000000000ULL / options.period_ns;
      std::cout << "Pin32相机触发已启动并由Pin33回采: 10 Hz, initial_pps="
                << initial_sequence << "；等待下一个PPS完成首个整秒验收" << std::endl;

      std::uint32_t last_sequence = initial_sequence;
      if (!fetch_pps(pps.get(), options.timeout_sec, event)) {
        throw std::runtime_error("初始锁相期间RTK PPS丢失");
      }
      double latency_ms = require_fresh_pps(event, last_sequence, options.maximum_latency_ms);
      if (event.info.assert_sequence != last_sequence + 1U) {
        throw std::runtime_error("初始锁相期间检测到PPS序号跳变");
      }
      last_sequence = event.info.assert_sequence;
      EdgeSnapshot edge = match_pps_edge(
        edge_capture, event, before_enable.count, expected_edges, options.maximum_latency_ms);
      const double initial_edge_phase_ms = physical_phase_ms(edge, event);
      std::uint64_t applied_period_ns = calculate_locked_period(options, edge, event);
      write_pwm_period(pwm_period.get(), applied_period_ns);
      write_ready_file(
        options, event, initial_sequence, initial_edge_phase_ms, latency_ms,
        initial_edge_phase_ms, expected_edges, applied_period_ns, edge);
      std::uint64_t previous_pps_edge_count = edge.count;

      while (stop_requested == 0) {
        if (!fetch_pps(pps.get(), options.timeout_sec, event)) {
          throw std::runtime_error("逐PPS校相期间RTK PPS丢失，切换保持模式");
        }
        if (stop_requested != 0) {
          break;
        }
        latency_ms = require_fresh_pps(
          event, last_sequence, options.maximum_latency_ms);
        if (event.info.assert_sequence != last_sequence + 1U) {
          throw std::runtime_error("逐PPS校相期间检测到PPS序号跳变");
        }
        last_sequence = event.info.assert_sequence;
        edge = match_pps_edge(
          edge_capture, event, previous_pps_edge_count, expected_edges,
          options.maximum_latency_ms);
        const double edge_phase_ms = physical_phase_ms(edge, event);
        applied_period_ns = calculate_locked_period(options, edge, event);
        write_pwm_period(pwm_period.get(), applied_period_ns);
        write_ready_file(
          options, event, initial_sequence, initial_edge_phase_ms, latency_ms,
          edge_phase_ms, expected_edges, applied_period_ns, edge);
        previous_pps_edge_count = edge.count;
      }
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
