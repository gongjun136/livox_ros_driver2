#include "lidar_dtc_monitor.h"
#include <cstdlib>
#include <iostream>
#include <set>

void Require(bool condition, const char* message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
int main() {
  using namespace livox_ros::dtc;
  const auto start = Clock::time_point{};
  Monitor monitor({}, start);
  Require(monitor.Evaluate(start).conditions[0] == kUnknown, "no evidence is not PASS");
  std::array<uint32_t, 8> hms{{0x01020002, 0x01110004}};
  monitor.ObserveHms(hms, start);
  Require(monitor.Evaluate(start).conditions[3] == kFail, "HMS high16 ID and OR mapping");
  Require(monitor.Evaluate(start).usability == kUnusable, "native fatal is unavailable despite system P2");
  hms[0] = 0;
  monitor.ObserveHms(hms, start);
  Require(monitor.Evaluate(start).conditions[3] == kFail, "clearing one reason does not clear another");
  hms = {{0x04010002,0x04050002,0x02010002}};
  monitor.ObserveHms(hms, start);
  monitor.ObserveWorkState(1, start);
  monitor.ObservePacket(start);
  monitor.ObserveCloud(10, 1, start);
  Require(monitor.Evaluate(start).usability == kUsable, "history/preheat warnings alone do not count fatal");
  const auto later = start + std::chrono::seconds(9);
  monitor.ObservePacket(later);
  monitor.ObserveCloud(10, 1, later);
  Require(monitor.Evaluate(later).usability == kUnusable, "frozen source stamp remains unavailable");
  Require(monitor.Evaluate(later).conditions[9] == kUnknown, "expired HMS does not recover");
  Monitor ptp({}, start);
  ptp.ObserveHms({{0x04050002, 0x04090002}}, start);
  ptp.ObserveWorkState(1, start);
  ptp.ObservePacket(start);
  ptp.ObserveCloud(100, 20, start);
  Require(ptp.Evaluate(start).conditions[1] == kFail, "GPS warning cannot be ignored without PTP evidence");
  // Captured healthy SANY PTP: sampling, advancing sync, about -5 us offset,
  // but firmware retains synchronization history and an unused GPS warning.
  ptp.ObserveTimeSync(1, 10000000000ULL, 9500000000ULL, -5000, start);
  Require(ptp.Evaluate(start).conditions[1] == kPass, "verified PTP excludes an unused GPS-source warning");
  Require(ptp.Evaluate(start).conditions[10] == kFail, "synchronization history remains visible in source DTC");
  for (uint32_t id : {0x0404u, 0x0407u, 0x0408u, 0x0409u}) {
    ptp.ObserveHms({{(id << 16) | 2}}, start);
    Require(ptp.Evaluate(start).conditions[1] == kPass, "only GPS/PPS warnings are excluded under verified PTP");
  }
  for (uint32_t id : {0x0402u, 0x0403u, 0x0406u, 0x040Au}) {
    ptp.ObserveHms({{0x04090002, (id << 16) | 2}}, start);
    Require(ptp.Evaluate(start).conditions[1] == kFail, "PTP/accuracy/conflicting-clock alarms remain active");
  }
  ptp.ObserveHms({{0x04090004}}, start);
  Require(ptp.Evaluate(start).conditions[1] == kFail && ptp.Evaluate(start).usability == kUnusable,
          "source filtering never masks a native fatal");
  ptp.ObserveHms({{0x04090003}}, start);
  Require(ptp.Evaluate(start).conditions[1] == kFail, "source filtering never masks a native error");
  ptp.ObserveHms({{0x04090002}}, start);
  ptp.ObserveTimeSync(2, 10000000000ULL, 9500000000ULL, -5000, start);
  Require(ptp.Evaluate(start).conditions[1] == kFail, "active GPS source keeps its GPS warning");
  ptp.ObserveTimeSync(1, 11000000000ULL, 10500000000ULL, -5000, start);
  Require(ptp.Evaluate(start).conditions[1] == kPass, "advancing healthy PTP recovers source selection");
  const auto expired = start + std::chrono::milliseconds(2501);
  ptp.ObserveHms({{0x04090002}}, expired);
  ptp.ObserveTimeSync(1, 11000000000ULL, 10500000000ULL, -5000, expired);
  Require(ptp.Evaluate(expired).conditions[1] == kFail, "repeated sync samples cannot refresh expired proof");
  ptp.ObserveTimeSync(1, 12000000000ULL, 11500000000ULL, 1000001, expired);
  Require(ptp.Evaluate(expired).conditions[1] == kFail, "large PTP offset cannot suppress warnings");
  ptp.ObserveTimeSync(1, 12000000000ULL, 11500000000ULL, -5000, expired);
  Require(ptp.Evaluate(expired).conditions[1] == kPass, "new valid proof restores selection");
  ptp.ObserveTimeSync(1, 10000000000ULL, 9500000000ULL, -5000, expired);
  Require(ptp.Evaluate(expired).conditions[1] == kFail, "backward synchronization time invalidates proof");
  ptp.ObserveTimeSync(1, 14000000000ULL, 10000000000ULL, -5000, expired);
  Require(ptp.Evaluate(expired).conditions[1] == kFail, "stale source time cannot suppress warnings");
  Require(Aggregate({{2,2,1,1}}) == std::array<uint8_t,2>{{kFail,kPass}}, "two failed");
  Monitor bounded_monitor({}, start);
  auto bounded_feed = [&](int seconds, int64_t offset = 1000000, uint32_t code = 0x04060002) {
    const auto t = start + std::chrono::seconds(seconds);
    bounded_monitor.ObserveWorkState(1, t);
    bounded_monitor.ObservePacket(t);
    bounded_monitor.ObserveCloud(1000 + seconds, 20, t);
    bounded_monitor.ObserveHms({{code}}, t);
    bounded_monitor.ObserveTimeSync(1, 10000000000ULL + seconds * 1000000000ULL,
        9500000000ULL + seconds * 1000000000ULL, offset, t);
    return bounded_monitor.Evaluate(t);
  };
  Require(!bounded_feed(0).bounded_time_sync_warning && !bounded_feed(1).bounded_time_sync_warning,
          "one sample or one advance cannot authorize a bounded warning");
  const auto proven = bounded_feed(2);
  Require(proven.bounded_time_sync_warning && proven.conditions[1] == kFail,
          "two advances within inclusive 1 ms allow advisory evidence but preserve source DTC FAIL");
  Require(!bounded_feed(3, 1000001).bounded_time_sync_warning, "1 ms plus 1 ns is blocking");
  Require(!bounded_feed(4, -1000000).bounded_time_sync_warning &&
          bounded_feed(5, -1000000).bounded_time_sync_warning, "negative inclusive bound requires fresh proof");
  for (uint32_t alarm : {0x04020002u, 0x04030002u, 0x040A0002u, 0x04060003u, 0x04060004u}) {
    bounded_monitor.ObserveHms({{0x04060002, alarm}}, start + std::chrono::seconds(5));
    Require(!bounded_monitor.Evaluate(start + std::chrono::seconds(5)).bounded_time_sync_warning,
            "loss, conflict, unknown time alarm or error/fatal is never advisory");
  }
  bounded_monitor.ObserveHms({{0x04060002}}, start + std::chrono::seconds(5));
  bounded_monitor.ObserveTimeSync(1, 16020000000ULL, 15520000000ULL, 0, start + std::chrono::seconds(6));
  Require(!bounded_monitor.Evaluate(start + std::chrono::seconds(6)).bounded_time_sync_warning,
          "forward device clock discontinuity revokes proof even with small reported offset");
  bounded_feed(7, 0); bounded_feed(8, 0);
  Require(bounded_feed(9, 0).bounded_time_sync_warning, "continuous new evidence recovers");
  const auto frozen = start + std::chrono::milliseconds(11501);
  bounded_monitor.ObserveHms({{0x04060002}}, frozen);
  bounded_monitor.ObservePacket(frozen); bounded_monitor.ObserveCloud(5000, 20, frozen);
  bounded_monitor.ObserveWorkState(1, frozen);
  bounded_monitor.ObserveTimeSync(1, 21501000000ULL, 18500000000ULL, 0, frozen);
  Require(!bounded_monitor.Evaluate(frozen).bounded_time_sync_warning, "frozen sync source cannot refresh proof");
  Require(Aggregate({{2,2,2,2}}) == std::array<uint8_t,2>{{kFail,kFail}}, "four means BOTH DTCs");
  Require(Aggregate({{2,0,1,1}}) == std::array<uint8_t,2>{{kUnknown,kPass}}, "unknown interval");
  std::set<uint32_t> codes;
  for (size_t role = 0; role < 4; ++role) for (auto code : Codes(role)) codes.insert(code);
  Require(codes.size() == 44, "44 unique single radar codes");
  std::cout << "lidar DTC checks passed\n";
}
