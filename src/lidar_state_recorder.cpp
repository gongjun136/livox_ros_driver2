#include "lidar_state_recorder.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#endif

#include "rapidjson/document.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

namespace livox_ros {
namespace raw_state {
namespace {
const char* const kFields[] = {"hms_code", "cur_work_state", "time_sync_type",
  "local_time_now", "last_sync_time", "time_offset", "core_temp", "powerup_cnt",
  "sn", "version_app", "version_loader", "version_hardware", "voltage", "uptime"};
const char* const kStatuses[] = {"missing", "valid", "invalid", "not_available", "no_response_length"};
const char* const kSources[] = {"startup", "state_push", "query", "shutdown", "error"};
const char* const kResults[] = {"not_applicable", "success", "submit_failed", "timeout",
  "sdk_error", "invalid_response", "pending"};
int64_t WallNow() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count(); }
int64_t SteadyNow() { return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count(); }
void Stamp(Record& record) {
  if (!record.wall_ns) record.wall_ns = WallNow();
  if (!record.steady_ns) record.steady_ns = SteadyNow();
}
bool SameSnapshot(const Snapshot& a, const Snapshot& b) {
  return a.status == b.status && a.hms == b.hms && a.work_state == b.work_state &&
      a.sync_type == b.sync_type && a.local_time == b.local_time && a.last_sync == b.last_sync &&
      a.time_offset == b.time_offset && a.core_temp == b.core_temp && a.powerup_count == b.powerup_count &&
      a.serial == b.serial && a.app_version == b.app_version && a.loader_version == b.loader_version &&
      a.hardware_version == b.hardware_version;
}
bool MakeDirectory(const std::string& path) {
  if (path.empty()) return false;
  for (size_t i = 1; i <= path.size(); ++i) {
    if (i != path.size() && path[i] != '/') continue;
    const std::string part = path.substr(0, i);
    if (::mkdir(part.c_str(), 0750) != 0 && errno != EEXIST) return false;
    struct stat st{};
    if (::stat(part.c_str(), &st) != 0 || !S_ISDIR(st.st_mode)) return false;
  }
  return true;
}
// This function runs exclusively on the writer thread. In particular setpriority
// uses the Linux TID, never the process ID, and never alters callback affinities.
std::string ConfigureWriter() {
#ifdef __linux__
  ::pthread_setname_np(::pthread_self(), "livox-state-log");
  struct sched_param normal{};
  if (::sched_setscheduler(0, SCHED_OTHER, &normal) != 0) return "normal_scheduler_set_failed";
  const auto tid = static_cast<id_t>(::syscall(SYS_gettid));
  const bool nice_ok = ::getpriority(PRIO_PROCESS, tid) >= 10 || ::setpriority(PRIO_PROCESS, tid, 10) == 0;
  if (!nice_ok) return "nice_set_failed";
  cpu_set_t allowed;
  if (::sched_getaffinity(0, sizeof(allowed), &allowed) != 0) return "affinity_read_failed";
  std::ifstream input("/sys/devices/system/cpu/isolated");
  std::string isolated;
  if (!input || !std::getline(input, isolated)) return "isolated_cpu_list_unavailable";
  std::istringstream list(isolated);
  std::string item;
  try {
    while (std::getline(list, item, ',')) {
      if (item.empty()) continue;
      const auto dash = item.find('-');
      const int first = std::stoi(item.substr(0, dash));
      const int last = dash == std::string::npos ? first : std::stoi(item.substr(dash + 1));
      if (first < 0 || last < first || last >= CPU_SETSIZE) return "isolated_cpu_list_invalid";
      for (int cpu = first; cpu <= last; ++cpu) CPU_CLR(cpu, &allowed);
    }
  } catch (...) { return "isolated_cpu_list_invalid"; }
  if (!CPU_COUNT(&allowed)) return "no_nonisolated_cpu_in_allowed_set";
  if (::sched_setaffinity(0, sizeof(allowed), &allowed) != 0) return "affinity_set_failed";
  return "nice10_nonisolated";
#else
  return "platform_not_linux";
#endif
}
}  // namespace

Snapshot::Snapshot() {
  status.fill(FieldStatus::Missing);
  status[Voltage] = status[Uptime] = FieldStatus::NotAvailable;
}
Snapshot Snapshot::QueryFields() {
  Snapshot fields;
  for (size_t i = CoreTemp; i <= HardwareVersion; ++i) fields.status[i] = FieldStatus::NoResponseLength;
  return fields;
}

bool ParseSdkJson(const char* json, size_t size, Snapshot& out) noexcept {
  out = Snapshot();
  try {
    rapidjson::Document doc;
    if (!json || size > 16384) return false;
    doc.Parse(json, size);
    if (doc.HasParseError() || !doc.IsObject()) return false;
    for (size_t i = 0; i < Voltage; ++i) {
      auto member = doc.FindMember(kFields[i]);
      if (member == doc.MemberEnd()) continue;
      const auto& v = member->value;
      bool valid = false;
      switch (i) {
        case Hms:
          valid = v.IsArray() && v.Size() == out.hms.size();
          if (valid) for (size_t j = 0; j < out.hms.size(); ++j) valid = valid && v[j].IsUint();
          if (valid) for (size_t j = 0; j < out.hms.size(); ++j) out.hms[j] = v[j].GetUint();
          break;
        case WorkState: case SyncType:
          valid = v.IsUint() && v.GetUint() <= 255;
          if (valid) (i == WorkState ? out.work_state : out.sync_type) = v.GetUint();
          break;
        case LocalTime: case LastSync:
          valid = v.IsUint64();
          if (valid) (i == LocalTime ? out.local_time : out.last_sync) = v.GetUint64();
          break;
        case TimeOffset: valid = v.IsInt64(); if (valid) out.time_offset = v.GetInt64(); break;
        case CoreTemp: valid = v.IsInt(); if (valid) out.core_temp = v.GetInt(); break;
        case PowerupCount: valid = v.IsUint(); if (valid) out.powerup_count = v.GetUint(); break;
        case Serial:
          valid = v.IsString() && v.GetStringLength() > 0 && v.GetStringLength() <= 16 &&
              std::memchr(v.GetString(), '\0', v.GetStringLength()) == nullptr;
          // Device serial numbers are printable ASCII. Invalid bytes must not
          // make the whole JSONL row unreadable (other fields remain useful).
          if (valid) for (size_t j = 0; j < v.GetStringLength(); ++j) {
            const auto byte = static_cast<unsigned char>(v.GetString()[j]);
            valid = valid && byte >= 0x20 && byte <= 0x7e;
          }
          if (valid) std::memcpy(out.serial.data(), v.GetString(), v.GetStringLength());
          break;
        case AppVersion: case LoaderVersion: case HardwareVersion: {
          auto& dest = i == AppVersion ? out.app_version : (i == LoaderVersion ? out.loader_version : out.hardware_version);
          valid = v.IsArray() && v.Size() == dest.size();
          if (valid) for (size_t j = 0; j < dest.size(); ++j) valid = valid && v[j].IsUint() && v[j].GetUint() <= 255;
          if (valid) for (size_t j = 0; j < dest.size(); ++j) dest[j] = v[j].GetUint();
          break;
        }
      }
      out.status[i] = valid ? FieldStatus::Valid : FieldStatus::Invalid;
    }
    return true;
  } catch (...) { return false; }
}

RecordQueue::RecordQueue(size_t capacity) : rows_(std::max<size_t>(1, capacity)) {}
bool RecordQueue::Push(Record record, bool filter_push) noexcept {
  std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
  if (!lock.owns_lock()) { ++dropped_; return false; }
  if (closed_) { ++dropped_; return false; }
  Stamp(record);
  if (filter_push && record.source == Source::StatePush && has_push_) {
    if (record.result == last_push_result_ && SameSnapshot(record.fields, last_push_)) { ++suppressed_; return false; }
    if (record.steady_ns - last_push_ns_ < 200000000) { ++dropped_; return false; }
  }
  if (size_ == rows_.size()) { ++dropped_; return false; }
  record.seq = ++sequence_;
  rows_[(head_ + size_) % rows_.size()] = record;
  ++size_;
  if (record.source == Source::StatePush) {
    has_push_ = true; last_push_ = record.fields; last_push_result_ = record.result; last_push_ns_ = record.steady_ns;
  }
  return true;
}
bool RecordQueue::Pop(Record& record) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!size_) return false;
  record = rows_[head_]; head_ = (head_ + 1) % rows_.size(); --size_;
  return true;
}
uint64_t RecordQueue::NextSequence() { std::lock_guard<std::mutex> lock(mutex_); return ++sequence_; }
void RecordQueue::Close() { std::lock_guard<std::mutex> lock(mutex_); closed_ = true; }

Recorder::Recorder(Options options) : options_(std::move(options)), queue_(options_.queue_capacity) {
  if (options_.directory.empty()) return;
  if (options_.role != "front" && options_.role != "rear" && options_.role != "left" && options_.role != "right") return;
  if (options_.file_bytes < 4096) throw std::invalid_argument("state recorder file size must be >= 4096");
  pid_ = static_cast<int>(::getpid());
  session_ = std::to_string(pid_) + "-" + std::to_string(WallNow()) + "-" + std::to_string(SteadyNow());
  path_ = options_.directory + "/" + options_.role + "_state.jsonl";
  enabled_ = true;
  accepting_ = true;
  Record record; record.source = Source::Startup;
  queue_.Push(record);
  thread_ = std::thread(&Recorder::Run, this);
}
Recorder::~Recorder() { Stop(); }
bool Recorder::RecordObservation(Record record) noexcept {
  if (!accepting_.load()) return false;
  const bool accepted = queue_.Push(record);
  if (accepted) wake_.notify_one();
  return accepted;
}
void Recorder::RecordJson(const char* json) noexcept {
  if (!accepting_.load()) return;
  Record record; record.source = Source::StatePush;
  const size_t size = json ? ::strnlen(json, 16385) : 0;
  if (!ParseSdkJson(json, size, record.fields)) {
    record.result = QueryResult::InvalidResponse;
    std::strcpy(record.detail.data(), "invalid_sdk_state_json");
  }
  RecordObservation(record);
}
void Recorder::RecordLifecycle(Source source, const char* detail) noexcept {
  Record record; record.source = source;
  if (detail) std::strncpy(record.detail.data(), detail, record.detail.size() - 1);
  RecordObservation(record);
}
void Recorder::Stop() {
  queue_.Close();
  accepting_ = false;
  wake_.notify_one();
  if (thread_.joinable()) thread_.join();
}
Counters Recorder::counters() const { return {queue_.dropped(), write_errors_.load(), queue_.suppressed()}; }

std::string Recorder::Serialize(const Record& row) const {
  rapidjson::StringBuffer buffer;
  rapidjson::Writer<rapidjson::StringBuffer> w(buffer);
  const auto& s = row.fields;
  w.StartObject();
  w.Key("schema"); w.Int(1); w.Key("kind"); w.String("livox_raw_state");
  w.Key("wall_ns"); w.Int64(row.wall_ns); w.Key("steady_ns"); w.Int64(row.steady_ns);
  w.Key("seq"); w.Uint64(row.seq); w.Key("pid"); w.Int(pid_);
  w.Key("session"); w.String(session_.c_str()); w.Key("role"); w.String(options_.role.c_str());
  w.Key("handle"); w.Uint(options_.handle); w.Key("ip"); w.String(options_.ip.c_str());
  w.Key("source"); w.String(kSources[static_cast<int>(row.source)]);
  w.Key("query_result"); w.String(kResults[static_cast<int>(row.result)]);
  w.Key("sdk_status"); if (row.has_sdk_status) w.Int(row.sdk_status); else w.Null();
  w.Key("ret_code"); if (row.has_ret_code) w.Int(row.ret_code); else w.Null();
  w.Key("detail"); w.String(row.detail.data());
  w.Key("fields"); w.StartObject();
  for (size_t i = 0; i < FieldCount; ++i) {
    w.Key(kFields[i]);
    if (s.status[i] != FieldStatus::Valid) { w.Null(); continue; }
    switch (i) {
      case Hms: w.StartArray(); for (auto v : s.hms) w.Uint(v); w.EndArray(); break;
      case WorkState: w.Uint(s.work_state); break;
      case SyncType: w.Uint(s.sync_type); break;
      case LocalTime: w.Uint64(s.local_time); break;
      case LastSync: w.Uint64(s.last_sync); break;
      case TimeOffset: w.Int64(s.time_offset); break;
      case CoreTemp: w.Int(s.core_temp); break;
      case PowerupCount: w.Uint(s.powerup_count); break;
      case Serial: w.String(s.serial.data()); break;
      case AppVersion: case LoaderVersion: case HardwareVersion: {
        const auto& version = i == AppVersion ? s.app_version : (i == LoaderVersion ? s.loader_version : s.hardware_version);
        w.StartArray(); for (auto v : version) w.Uint(v); w.EndArray(); break;
      }
      default: w.Null();
    }
  }
  w.EndObject();
  w.Key("field_status"); w.StartObject();
  for (size_t i = 0; i < FieldCount; ++i) { w.Key(kFields[i]); w.String(kStatuses[static_cast<int>(s.status[i])]); }
  w.EndObject();
  w.Key("field_sources"); w.StartObject();
  for (size_t i = 0; i < FieldCount; ++i) {
    w.Key(kFields[i]); w.String(s.status[i] == FieldStatus::Valid || s.status[i] == FieldStatus::Invalid ?
        kSources[static_cast<int>(row.source)] : "unavailable");
  }
  w.EndObject();
  w.Key("core_temp_unit"); w.String("sdk_raw_unit_unspecified");
  w.Key("dropped_total"); w.Uint64(queue_.dropped());
  w.Key("write_errors_total"); w.Uint64(write_errors_.load());
  w.Key("suppressed_push_total"); w.Uint64(queue_.suppressed());
  w.Key("writer_scheduling"); w.String(scheduling_.c_str());
  w.EndObject();
  return std::string(buffer.GetString(), buffer.GetSize()) + '\n';
}
void Recorder::WriteError() {
  const auto count = ++write_errors_;
  if (count == 1) std::fprintf(stderr, "LiDAR raw state recorder write failed (%s); driver continues, errors counted\n", path_.c_str());
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}
bool Recorder::Open() {
  if (fd_ >= 0) return true;
  if (!MakeDirectory(options_.directory)) return false;
  fd_ = ::open(path_.c_str(), O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC | O_NONBLOCK | O_NOFOLLOW, 0640);
  if (fd_ < 0) return false;
  struct stat st{};
  if (::fstat(fd_, &st) != 0 || !S_ISREG(st.st_mode)) { ::close(fd_); fd_ = -1; return false; }
  file_size_ = static_cast<size_t>(st.st_size);
  // A killed process may leave its final row partial. Start a fresh rotation so
  // the next session can never be appended to that incomplete JSON object.
  if (file_size_) return Rotate();
  return true;
}
bool Recorder::Rotate() {
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
  if (::unlink((path_ + ".3").c_str()) != 0 && errno != ENOENT) return false;
  for (int i = 2; i >= 0; --i) {
    const auto from = i ? path_ + "." + std::to_string(i) : path_;
    const auto to = path_ + "." + std::to_string(i + 1);
    if (::rename(from.c_str(), to.c_str()) != 0 && errno != ENOENT) return false;
  }
  fd_ = ::open(path_.c_str(), O_WRONLY | O_APPEND | O_CREAT | O_CLOEXEC | O_NONBLOCK | O_NOFOLLOW, 0640);
  file_size_ = 0;
  if (fd_ < 0) return false;
  struct stat st{};
  if (::fstat(fd_, &st) != 0 || !S_ISREG(st.st_mode)) { ::close(fd_); fd_ = -1; return false; }
  return true;
}
bool Recorder::Write(const Record& record) {
  const auto line = Serialize(record);
  if (line.size() > options_.file_bytes) return false;
  if (!Open()) return false;
  if (file_size_ + line.size() > options_.file_bytes && !Rotate()) return false;
  size_t offset = 0;
  while (offset < line.size()) {
    const auto count = ::write(fd_, line.data() + offset, line.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) return false;
    offset += static_cast<size_t>(count);
  }
  file_size_ += line.size();
  return true;
}
void Recorder::Run() noexcept {
  try {
    scheduling_ = ConfigureWriter();
    // If pinning cannot exclude isolated CPUs, disable I/O rather than compete
    // with real-time workers. The recorder failure is visible on stderr.
    if (scheduling_ != "nice10_nonisolated" && scheduling_ != "platform_not_linux") {
      std::fprintf(stderr, "LiDAR raw state recorder disabled: %s\n", scheduling_.c_str());
      ++write_errors_; accepting_ = false; return;
    }
    while (true) {
      Record record;
      if (queue_.Pop(record)) {
        if (!Write(record)) WriteError();
        continue;
      }
      if (!accepting_.load()) {
        // Stop closes the queue before clearing accepting_. A producer may have
        // enqueued between our first empty Pop and that close; check once more.
        if (!queue_.Pop(record)) break;
        if (!Write(record)) WriteError();
        continue;
      }
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_.wait_for(lock, std::chrono::milliseconds(200));
    }
    Record shutdown; shutdown.source = Source::Shutdown; Stamp(shutdown);
    shutdown.seq = queue_.NextSequence();
    if (!Write(shutdown)) WriteError();
  } catch (...) { WriteError(); accepting_ = false; }
  if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

}  // namespace raw_state
}  // namespace livox_ros
