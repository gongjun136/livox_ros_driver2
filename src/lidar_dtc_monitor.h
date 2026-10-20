#ifndef LIVOX_LIDAR_DTC_MONITOR_H_
#define LIVOX_LIDAR_DTC_MONITOR_H_

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>

namespace livox_ros {
namespace dtc {
using Clock = std::chrono::steady_clock;
enum Condition : uint8_t { kUnknown = 0, kPass = 1, kFail = 2 };
enum Usability : uint8_t { kUnavailableUnknown = 0, kUsable = 1, kUnusable = 2 };
constexpr uint32_t kMultipleFailed = 0x10F00001;
constexpr uint32_t kAllFailed = 0x10F00002;
constexpr size_t kCategories = 11;

inline const std::array<uint32_t, kCategories>& Codes(size_t role) {
  // Category order: data, time, scan, temperature, voltage, upgrade, IMU,
  // parameters, hardware, Ethernet history, synchronization history.
  static const std::array<std::array<uint32_t, kCategories>, 4> codes{{
      {{0x110476C4,0x110476C5,0x110476C7,0x110476C8,0x110476CB,0x110476CC,
        0x110476CD,0x110476CE,0x110476CF,0x110476D0,0x110476D1}},
      {{0x11037660,0x11037661,0x11037663,0x11037664,0x11037667,0x11037668,
        0x11037669,0x1103766A,0x1103766B,0x1103766C,0x1103766D}},
      {{0x11017598,0x11017599,0x1101759B,0x1101759C,0x1101759F,0x110175A0,
        0x110175A1,0x110175A2,0x110175A3,0x110175A4,0x110175A5}},
      {{0x110275FC,0x110275FD,0x110275FF,0x11027600,0x11027603,0x11027604,
        0x11027605,0x11027606,0x11027607,0x11027608,0x11027609}}
  }};
  return codes.at(role);
}

inline int Category(uint16_t id) {
  if (id == 0x0104 || id == 0x0304) return 0;
  if (id == 0x0402 || id == 0x0403 || id == 0x0404 || (id >= 0x0406 && id <= 0x040A)) return 1;
  if ((id >= 0x0210 && id <= 0x0219) || id == 0x021C) return 2;
  if (id == 0x0102 || id == 0x0103 || id == 0x0111 || id == 0x0112 || id == 0x0114 || id == 0x0115) return 3;
  if (id == 0x0116) return 4;
  if (id == 0x0105) return 5;
  if (id == 0x0113) return 6;
  if (id == 0x0117) return 7;
  if (id == 0x0118) return 8;
  if (id == 0x0401) return 9;
  if (id == 0x0405) return 10;
  return -1;
}

inline uint64_t AgeUs(Clock::time_point now, Clock::time_point then, bool seen) {
  if (!seen) return std::numeric_limits<uint64_t>::max();
  return static_cast<uint64_t>(std::max<int64_t>(0,
      std::chrono::duration_cast<std::chrono::microseconds>(now - then).count()));
}

struct MonitorConfig {
  std::chrono::milliseconds data_timeout{500};
  std::chrono::milliseconds state_timeout{2500};
  std::chrono::milliseconds startup_grace{8000};
  size_t min_valid_points = 1;
  int64_t max_sync_offset_ns = 1000000;  // Require verified PTP within 1 ms before excluding GPS-only warnings.
};

struct Snapshot {
  std::array<uint8_t, kCategories> conditions{};
  uint32_t hms_seq = 0;
  uint64_t hms_age_us = std::numeric_limits<uint64_t>::max();
  uint8_t usability = kUnavailableUnknown;
  bool bounded_time_sync_warning = false;
  int64_t time_sync_offset_ns = 0;
  uint64_t time_sync_evidence_age_us = std::numeric_limits<uint64_t>::max();
};

class Monitor {
 public:
  explicit Monitor(MonitorConfig config = {}, Clock::time_point start = Clock::now())
      : config_(config), start_(start) {}

  void ObserveHms(const std::array<uint32_t, 8>& codes, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    hms_ = codes;
    hms_time_ = now;
    hms_seen_ = true;
    ++hms_seq_;
  }
  void ObserveWorkState(uint8_t state, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    work_state_ = state;
    state_time_ = now;
    state_seen_ = true;
  }
  void ObserveTimeSync(uint8_t type, uint64_t local_time, uint64_t last_sync,
                       int64_t offset, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Separate proof for the consumer's bounded warning policy. Keep the
    // original source DTC and GPS-source verification semantics unchanged.
    const auto host_delta = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now - bounded_sample_time_).count();
    const bool clock_continuous = bounded_seen_ && local_time >= bounded_local_ &&
        host_delta >= 0 &&
        std::abs(static_cast<double>(local_time - bounded_local_) - host_delta) <= 1000000.0;
    const bool bounded = type == 1 && last_sync != 0 && local_time >= last_sync &&
        local_time - last_sync <= 2500000000ULL && offset >= -1000000 && offset <= 1000000 &&
        (!bounded_seen_ || last_sync >= bounded_last_sync_);
    if (!bounded || !clock_continuous) bounded_advances_ = 0;
    else if (last_sync > bounded_last_sync_) {
      bounded_advances_ = std::min(2u, bounded_advances_ + 1);
      bounded_advance_time_ = now;
    }
    bounded_seen_ = true;
    bounded_local_ = local_time;
    bounded_last_sync_ = last_sync;
    bounded_sample_time_ = now;
    bounded_offset_ = offset;
    const auto ttl_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(config_.state_timeout).count();
    const bool healthy = type == 1 && last_sync != 0 && local_time >= last_sync &&
        local_time - last_sync <= static_cast<uint64_t>(ttl_ns) &&
        offset >= -config_.max_sync_offset_ns && offset <= config_.max_sync_offset_ns;
    if (!healthy || (sync_seen_ && last_sync < last_sync_)) {
      ptp_verified_ = false;
      return;
    }
    // Repeated status queries must not refresh a frozen synchronization source.
    if (!sync_seen_ || last_sync > last_sync_) {
      last_sync_ = last_sync;
      sync_time_ = now;
      sync_seen_ = ptp_verified_ = true;
    }
  }
  void ObservePacket(Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    packet_time_ = now;
    packet_seen_ = true;
  }
  void ObserveCloud(uint64_t stamp, size_t valid_points, Clock::time_point now = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stamp > valid_stamp_ && valid_points >= config_.min_valid_points) {
      valid_stamp_ = stamp;
      valid_time_ = now;
      valid_seen_ = true;
    }
  }
  Snapshot Evaluate(Clock::time_point now = Clock::now()) const {
    std::lock_guard<std::mutex> lock(mutex_);
    Snapshot result;
    result.hms_seq = hms_seq_;
    result.hms_age_us = AgeUs(now, hms_time_, hms_seen_);
    const bool hms_fresh = hms_seen_ && now - hms_time_ <= config_.state_timeout;
    const bool healthy_ptp = ptp_verified_ && sync_seen_ && now - sync_time_ <= config_.state_timeout;
    result.time_sync_offset_ns = bounded_offset_;
    result.time_sync_evidence_age_us = AgeUs(now, bounded_advance_time_, bounded_advances_ >= 2);
    bool bounded_warning = false, incompatible_sync_alarm = false;
    bool fatal = false;
    if (hms_fresh) {
      result.conditions.fill(kPass);
      for (uint32_t code : hms_) {
        if (code == 0) continue;
        const uint16_t id = static_cast<uint16_t>(code >> 16);
        const uint8_t level = static_cast<uint8_t>(code & 0xFF);
        fatal = fatal || level == 4;
        const int category = Category(id);
        if (category == 1) {
          // Only an accuracy warning may be advisory. Never downgrade
          // synchronization loss, clock conflict, unknown codes, errors/fatal.
          bounded_warning |= id == 0x0406 && level == 2;
          incompatible_sync_alarm |= level != 2 ||
              !(id == 0x0406 || id == 0x0404 || (id >= 0x0407 && id <= 0x0409));
        }
        // GPS/PPS warnings do not describe the active clock when independent,
        // advancing PTP status proves it healthy. Never mask PTP/conflict codes,
        // a native error/fatal, or warnings without fresh verification.
        if (healthy_ptp && level == 2 && (id == 0x0404 || (id >= 0x0407 && id <= 0x0409))) continue;
        if (category >= 0) result.conditions[category] = kFail;
      }
    }
    const bool state_fresh = state_seen_ && now - state_time_ <= config_.state_timeout;
    const bool valid_fresh = valid_seen_ && now - valid_time_ <= config_.data_timeout;
    const bool packet_fresh = packet_seen_ && now - packet_time_ <= config_.data_timeout;
    if (fatal || (state_fresh && work_state_ == 4)) {
      result.usability = kUnusable;
    } else if ((!valid_fresh || !packet_fresh) && (valid_seen_ || now - start_ >= config_.startup_grace)) {
      result.usability = kUnusable;
    } else if (valid_fresh && packet_fresh && hms_fresh && state_fresh && work_state_ == 1) {
      result.usability = kUsable;
    } else if (state_fresh && work_state_ != 1 && now - start_ >= config_.startup_grace) {
      result.usability = kUnusable;
    }
    result.bounded_time_sync_warning = hms_fresh && bounded_warning && !incompatible_sync_alarm &&
        result.usability == kUsable && bounded_advances_ >= 2 &&
        result.time_sync_evidence_age_us <= 2500000 &&
        now - bounded_advance_time_ <= config_.state_timeout &&
        bounded_offset_ >= -1000000 && bounded_offset_ <= 1000000;
    return result;
  }

 private:
  MonitorConfig config_;
  Clock::time_point start_, hms_time_, state_time_, packet_time_, valid_time_;
  Clock::time_point sync_time_;
  mutable std::mutex mutex_;
  std::array<uint32_t, 8> hms_{};
  uint64_t valid_stamp_ = 0;
  uint64_t last_sync_ = 0;
  uint32_t hms_seq_ = 0;
  uint8_t work_state_ = 0;
  bool hms_seen_ = false, state_seen_ = false, packet_seen_ = false, valid_seen_ = false;
  bool sync_seen_ = false, ptp_verified_ = false;
  bool bounded_seen_ = false;
  unsigned bounded_advances_ = 0;
  uint64_t bounded_local_ = 0, bounded_last_sync_ = 0;
  int64_t bounded_offset_ = 0;
  Clock::time_point bounded_sample_time_, bounded_advance_time_;
};

inline std::array<uint8_t, 2> Aggregate(const std::array<uint8_t, 4>& states) {
  size_t failed = 0, unknown = 0;
  for (uint8_t state : states) {
    failed += state == kUnusable;
    unknown += state == kUnavailableUnknown;
  }
  return {{static_cast<uint8_t>(failed >= 2 ? kFail : failed + unknown < 2 ? kPass : kUnknown),
           static_cast<uint8_t>(failed == 4 ? kFail : failed + unknown < 4 ? kPass : kUnknown)}};
}
}  // namespace dtc
}  // namespace livox_ros
#endif
