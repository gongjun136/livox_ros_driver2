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
  Require(Aggregate({{2,2,1,1}}) == std::array<uint8_t,2>{{kFail,kPass}}, "two failed");
  Require(Aggregate({{2,2,2,2}}) == std::array<uint8_t,2>{{kFail,kFail}}, "four means BOTH DTCs");
  Require(Aggregate({{2,0,1,1}}) == std::array<uint8_t,2>{{kUnknown,kPass}}, "unknown interval");
  std::set<uint32_t> codes;
  for (size_t role = 0; role < 4; ++role) for (auto code : Codes(role)) codes.insert(code);
  Require(codes.size() == 44, "44 unique single radar codes");
  std::cout << "lidar DTC checks passed\n";
}
