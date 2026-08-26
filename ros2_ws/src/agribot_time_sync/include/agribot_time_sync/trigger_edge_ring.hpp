// Copyright 2026 cgz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#ifndef AGRIBOT_TIME_SYNC__TRIGGER_EDGE_RING_HPP_
#define AGRIBOT_TIME_SYNC__TRIGGER_EDGE_RING_HPP_

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <ctime>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace agribot_time_sync
{

constexpr std::uint64_t kTriggerEdgeRingMagic = 0x4147524945444745ULL;
constexpr std::uint32_t kTriggerEdgeRingVersion = 1U;
constexpr std::uint32_t kTriggerEdgeRingCapacity = 256U;

struct TriggerEdge
{
  std::uint64_t sequence;
  std::int64_t timestamp_ns;
  std::uint32_t kernel_sequence;
  std::uint32_t reserved;
};

struct alignas(64) TriggerEdgeRingStorage
{
  std::uint64_t magic;
  std::uint32_t version;
  std::uint32_t capacity;
  std::uint64_t generation;
  alignas(8) std::uint64_t write_sequence;
  std::uint64_t reserved[3];
  TriggerEdge edges[kTriggerEdgeRingCapacity];
};

class TriggerEdgeRingWriter
{
public:
  explicit TriggerEdgeRingWriter(const std::string & path)
  : descriptor_(open(path.c_str(), O_RDWR | O_CREAT | O_CLOEXEC, 0644))
  {
    if (descriptor_ < 0) {
      throw std::runtime_error("无法创建触发沿缓冲" + path + ": " + std::strerror(errno));
    }
    if (fchmod(descriptor_, 0644) != 0 ||
      ftruncate(descriptor_, static_cast<off_t>(sizeof(TriggerEdgeRingStorage))) != 0)
    {
      const std::string error = std::strerror(errno);
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("无法初始化触发沿缓冲" + path + ": " + error);
    }
    void * mapping = mmap(
      nullptr, sizeof(TriggerEdgeRingStorage), PROT_READ | PROT_WRITE, MAP_SHARED,
      descriptor_, 0);
    if (mapping == MAP_FAILED) {
      const std::string error = std::strerror(errno);
      close(descriptor_);
      descriptor_ = -1;
      throw std::runtime_error("无法映射触发沿缓冲" + path + ": " + error);
    }
    storage_ = static_cast<TriggerEdgeRingStorage *>(mapping);
    std::memset(storage_, 0, sizeof(*storage_));
    storage_->magic = kTriggerEdgeRingMagic;
    storage_->version = kTriggerEdgeRingVersion;
    storage_->capacity = kTriggerEdgeRingCapacity;
    timespec now{};
    if (clock_gettime(CLOCK_REALTIME, &now) != 0) {
      throw std::runtime_error("无法生成触发沿缓冲代号: " + std::string(std::strerror(errno)));
    }
    storage_->generation = static_cast<std::uint64_t>(now.tv_sec) * 1000000000ULL +
      static_cast<std::uint64_t>(now.tv_nsec);
    __atomic_store_n(&storage_->write_sequence, 0U, __ATOMIC_RELEASE);
  }

  ~TriggerEdgeRingWriter()
  {
    if (storage_ != nullptr) {
      munmap(storage_, sizeof(*storage_));
    }
    if (descriptor_ >= 0) {
      close(descriptor_);
    }
  }

  TriggerEdgeRingWriter(const TriggerEdgeRingWriter &) = delete;
  TriggerEdgeRingWriter & operator=(const TriggerEdgeRingWriter &) = delete;

  std::uint64_t write(const std::int64_t timestamp_ns, const std::uint32_t kernel_sequence)
  {
    const std::uint64_t sequence =
      __atomic_load_n(&storage_->write_sequence, __ATOMIC_ACQUIRE) + 1U;
    TriggerEdge & edge = storage_->edges[(sequence - 1U) % kTriggerEdgeRingCapacity];
    __atomic_store_n(&edge.sequence, 0U, __ATOMIC_RELEASE);
    edge.timestamp_ns = timestamp_ns;
    edge.kernel_sequence = kernel_sequence;
    __atomic_store_n(&edge.sequence, sequence, __ATOMIC_RELEASE);
    __atomic_store_n(&storage_->write_sequence, sequence, __ATOMIC_RELEASE);
    return sequence;
  }

  std::uint64_t generation() const {return storage_->generation;}

private:
  int descriptor_{-1};
  TriggerEdgeRingStorage * storage_{nullptr};
};

class TriggerEdgeRingReader
{
public:
  TriggerEdgeRingReader() = default;
  ~TriggerEdgeRingReader() {close_mapping();}

  TriggerEdgeRingReader(const TriggerEdgeRingReader &) = delete;
  TriggerEdgeRingReader & operator=(const TriggerEdgeRingReader &) = delete;

  void open_path(const std::string & path)
  {
    close_mapping();
    descriptor_ = open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (descriptor_ < 0) {
      throw std::runtime_error("无法打开触发沿缓冲" + path + ": " + std::strerror(errno));
    }
    struct stat status {};
    if (fstat(descriptor_, &status) != 0 ||
      status.st_size < static_cast<off_t>(sizeof(TriggerEdgeRingStorage)))
    {
      const std::string error = errno == 0 ? "文件尺寸无效" : std::strerror(errno);
      close_mapping();
      throw std::runtime_error("触发沿缓冲无效" + path + ": " + error);
    }
    void * mapping = mmap(
      nullptr, sizeof(TriggerEdgeRingStorage), PROT_READ, MAP_SHARED, descriptor_, 0);
    if (mapping == MAP_FAILED) {
      const std::string error = std::strerror(errno);
      storage_ = nullptr;
      close_mapping();
      throw std::runtime_error("无法映射触发沿缓冲" + path + ": " + error);
    }
    storage_ = static_cast<const TriggerEdgeRingStorage *>(mapping);
    if (storage_->magic != kTriggerEdgeRingMagic ||
      storage_->version != kTriggerEdgeRingVersion ||
      storage_->capacity != kTriggerEdgeRingCapacity)
    {
      close_mapping();
      throw std::runtime_error("触发沿缓冲格式不兼容: " + path);
    }
  }

  bool is_open() const {return storage_ != nullptr;}

  std::uint64_t generation() const
  {
    return storage_ == nullptr ? 0U : storage_->generation;
  }

  std::uint64_t latest_sequence() const
  {
    return storage_ == nullptr ? 0U :
      __atomic_load_n(&storage_->write_sequence, __ATOMIC_ACQUIRE);
  }

  bool read(const std::uint64_t sequence, TriggerEdge & result) const
  {
    if (storage_ == nullptr || sequence == 0U) {
      return false;
    }
    const std::uint64_t latest = latest_sequence();
    if (sequence > latest || latest - sequence >= kTriggerEdgeRingCapacity) {
      return false;
    }
    const TriggerEdge & edge = storage_->edges[(sequence - 1U) % kTriggerEdgeRingCapacity];
    const std::uint64_t first = __atomic_load_n(&edge.sequence, __ATOMIC_ACQUIRE);
    if (first != sequence) {
      return false;
    }
    result.timestamp_ns = edge.timestamp_ns;
    result.kernel_sequence = edge.kernel_sequence;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    const std::uint64_t second = __atomic_load_n(&edge.sequence, __ATOMIC_ACQUIRE);
    if (second != sequence) {
      return false;
    }
    result.sequence = sequence;
    return true;
  }

private:
  void close_mapping()
  {
    if (storage_ != nullptr) {
      munmap(const_cast<TriggerEdgeRingStorage *>(storage_), sizeof(*storage_));
      storage_ = nullptr;
    }
    if (descriptor_ >= 0) {
      close(descriptor_);
      descriptor_ = -1;
    }
  }

  int descriptor_{-1};
  const TriggerEdgeRingStorage * storage_{nullptr};
};

struct TriggerEdgeMatchResult
{
  bool matched{false};
  TriggerEdge edge{};
  std::int64_t receipt_delay_ns{0};
};

class TriggerEdgeMatcher
{
public:
  TriggerEdgeMatcher(
    const std::int64_t maximum_receipt_delay_ns = 90000000,
    const std::int64_t maximum_future_ns = 2000000)
  : maximum_receipt_delay_ns_(std::max<std::int64_t>(maximum_receipt_delay_ns, 0)),
    maximum_future_ns_(std::max<std::int64_t>(maximum_future_ns, 0))
  {
  }

  TriggerEdgeMatchResult match(
    const std::uint32_t frame_number, const std::int64_t receipt_ns,
    const TriggerEdgeRingReader & reader)
  {
    const std::uint64_t generation = reader.generation();
    if (generation == 0U) {
      return {};
    }
    if (generation_ != generation) {
      reset();
      generation_ = generation;
    }

    TriggerEdge candidate;
    if (initialized_) {
      const std::uint32_t frame_delta = frame_number - last_frame_number_;
      if (frame_delta > 0U && frame_delta < kTriggerEdgeRingCapacity &&
        reader.read(last_edge_sequence_ + frame_delta, candidate) &&
        valid_time(candidate.timestamp_ns, receipt_ns))
      {
        return accept(frame_number, receipt_ns, candidate);
      }
    }

    const std::uint64_t latest = reader.latest_sequence();
    const std::uint64_t oldest = latest > kTriggerEdgeRingCapacity ?
      latest - kTriggerEdgeRingCapacity + 1U : 1U;
    for (std::uint64_t sequence = latest; sequence >= oldest && sequence > 0U; --sequence) {
      if (!reader.read(sequence, candidate)) {
        continue;
      }
      if (candidate.timestamp_ns > receipt_ns + maximum_future_ns_) {
        continue;
      }
      if (valid_time(candidate.timestamp_ns, receipt_ns)) {
        return accept(frame_number, receipt_ns, candidate);
      }
      if (receipt_ns - candidate.timestamp_ns > maximum_receipt_delay_ns_) {
        break;
      }
    }
    return {};
  }

  void reset()
  {
    initialized_ = false;
    last_frame_number_ = 0U;
    last_edge_sequence_ = 0U;
  }

private:
  bool valid_time(const std::int64_t edge_ns, const std::int64_t receipt_ns) const
  {
    const std::int64_t delay = receipt_ns - edge_ns;
    return delay >= -maximum_future_ns_ && delay <= maximum_receipt_delay_ns_;
  }

  TriggerEdgeMatchResult accept(
    const std::uint32_t frame_number, const std::int64_t receipt_ns,
    const TriggerEdge & edge)
  {
    initialized_ = true;
    last_frame_number_ = frame_number;
    last_edge_sequence_ = edge.sequence;
    TriggerEdgeMatchResult result;
    result.matched = true;
    result.edge = edge;
    result.receipt_delay_ns = receipt_ns - edge.timestamp_ns;
    return result;
  }

  std::int64_t maximum_receipt_delay_ns_;
  std::int64_t maximum_future_ns_;
  std::uint64_t generation_{0U};
  bool initialized_{false};
  std::uint32_t last_frame_number_{0U};
  std::uint64_t last_edge_sequence_{0U};
};

}  // namespace agribot_time_sync

#endif  // AGRIBOT_TIME_SYNC__TRIGGER_EDGE_RING_HPP_
