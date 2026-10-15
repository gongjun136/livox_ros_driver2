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
  Require(Aggregate({{2,2,2,2}}) == std::array<uint8_t,2>{{kFail,kFail}}, "four means BOTH DTCs");
  Require(Aggregate({{2,0,1,1}}) == std::array<uint8_t,2>{{kUnknown,kPass}}, "unknown interval");
  std::set<uint32_t> codes;
  for (size_t role = 0; role < 4; ++role) for (auto code : Codes(role)) codes.insert(code);
  Require(codes.size() == 44, "44 unique single radar codes");
  std::cout << "lidar DTC checks passed\n";
}
