#ifndef LIVOX_ROS_DRIVER_LIDAR_STATE_RECORDER_H_
#define LIVOX_ROS_DRIVER_LIDAR_STATE_RECORDER_H_

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace livox_ros {
namespace raw_state {

enum FieldId { Hms, WorkState, SyncType, LocalTime, LastSync, TimeOffset,
  CoreTemp, PowerupCount, Serial, AppVersion, LoaderVersion, HardwareVersion,
  Voltage, Uptime, FieldCount };
enum class FieldStatus { Missing, Valid, Invalid, NotAvailable, NoResponseLength };
enum class Source { Startup, StatePush, Query, Shutdown, Error };
enum class QueryResult { NotApplicable, Success, SubmitFailed, Timeout, SdkError,
  InvalidResponse, Pending };

// One observation only: missing fields are never filled from an older callback.
struct Snapshot {
  Snapshot();
  static Snapshot QueryFields();
  std::array<FieldStatus, FieldCount> status;
  std::array<uint32_t, 8> hms{};
  uint8_t work_state = 0, sync_type = 0;
  uint64_t local_time = 0, last_sync = 0;
  int64_t time_offset = 0;
  int32_t core_temp = 0;
  uint32_t powerup_count = 0;
  std::array<char, 17> serial{};
  std::array<uint8_t, 4> app_version{}, loader_version{}, hardware_version{};
};

// Parses only the SDK's already materialized JSON, never a lengthless TLV buffer.
bool ParseSdkJson(const char* json, size_t size, Snapshot& snapshot) noexcept;

struct Record {
  Source source = Source::Query;
  QueryResult result = QueryResult::NotApplicable;
  Snapshot fields;
  int sdk_status = 0, ret_code = 0;
  bool has_sdk_status = false, has_ret_code = false;
  int64_t wall_ns = 0, steady_ns = 0;
  uint64_t seq = 0;
  std::array<char, 96> detail{};
};

struct Counters { uint64_t dropped = 0, write_errors = 0, suppressed_push = 0; };

// Fixed capacity storage; no allocation and no blocking mutex acquisition on Push.
class RecordQueue {
 public:
  explicit RecordQueue(size_t capacity);
  bool Push(Record record, bool filter_push = true) noexcept;
  bool Pop(Record& record);
  void Close();  // synchronizes with producers; already accepted rows remain readable
  uint64_t dropped() const { return dropped_.load(); }
  uint64_t suppressed() const { return suppressed_.load(); }
  uint64_t NextSequence();  // writer only, after producers have stopped
 private:
  std::vector<Record> rows_;
  std::mutex mutex_;
  size_t head_ = 0, size_ = 0;
  uint64_t sequence_ = 0;
  std::atomic<uint64_t> dropped_{0}, suppressed_{0};
  bool has_push_ = false;
  bool closed_ = false;
  Snapshot last_push_;
  QueryResult last_push_result_ = QueryResult::NotApplicable;
  int64_t last_push_ns_ = 0;
};

class Recorder {
 public:
  struct Options {
    std::string directory, role, ip;
    uint32_t handle = 0;
    size_t file_bytes = 8 * 1024 * 1024;  // current + .1 + .2 + .3 per role
    size_t queue_capacity = 128;
  };
  explicit Recorder(Options options);
  ~Recorder();
  Recorder(const Recorder&) = delete;
  Recorder& operator=(const Recorder&) = delete;
  bool RecordObservation(Record record) noexcept;
  void RecordJson(const char* json) noexcept;
  void RecordLifecycle(Source source, const char* detail) noexcept;
  void Stop();  // closes the queue before draining; safe against late submissions
  Counters counters() const;
  bool enabled() const { return enabled_; }
 private:
  void Run() noexcept;
  bool Open();
  bool Write(const Record& record);
  bool Rotate();
  void WriteError();
  std::string Serialize(const Record& record) const;
  Options options_;
  RecordQueue queue_;
  std::atomic<bool> accepting_{false};
  std::atomic<uint64_t> write_errors_{0};
  bool enabled_ = false;
  std::thread thread_;
  std::mutex wake_mutex_;
  std::condition_variable wake_;
  std::string session_, path_, scheduling_;
  int pid_ = 0, fd_ = -1;
  size_t file_size_ = 0;
};

}  // namespace raw_state
}  // namespace livox_ros
#endif
