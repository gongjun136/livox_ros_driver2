#include "lidar_dtc_reporting.h"
#include <iostream>
#include <stdexcept>

using namespace livox_ros;
using Heartbeat = diagnostic_monitor_interfaces::msg::NodeHeartbeat;
using Diagnostic = livox_ros_driver2::msg::LidarDiagnosticState;

namespace {
void Require(bool value, const char* reason) {
  if (!value) throw std::runtime_error(reason);
}
auto At(int ms) { return dtc::Clock::time_point{} + std::chrono::milliseconds(ms); }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("driver_dtc_reporting_test");
  const std::array<std::string, 4> roles{{"front", "rear", "left", "right"}};
  const std::array<uint16_t, 4> ids{{7, 12, 8, 9}};
  const std::array<uint32_t, 4> bases{{0x110476C4, 0x11037660, 0x11017598, 0x110275FC}};
  std::array<std::unique_ptr<dtc::Monitor>, 4> monitors;
  std::array<std::unique_ptr<functional_safety::HeartbeatPublisher>, 4> heartbeats;
  std::array<rclcpp::Publisher<Diagnostic>::SharedPtr, 4> publishers;
  std::array<rclcpp::Subscription<Heartbeat>::SharedPtr, 4> heartbeat_subs;
  std::array<rclcpp::Subscription<Diagnostic>::SharedPtr, 4> diagnostic_subs;
  std::array<Heartbeat, 4> latest;
  std::array<Diagnostic, 4> diagnostics;
  std::array<size_t, 4> received{}, diagnostic_received{};
  std::array<uint32_t, 4> sequences{};
  std::array<std::vector<uint32_t>, 4> expected;
  for (size_t role = 0; role < 4; ++role) {
    const auto heartbeat_topic = "/diagnostics/heartbeat/livox_" + roles[role];
    const auto state_topic = "/sensors/livox/" + roles[role] + "/diagnostic_state";
    heartbeat_subs[role] = node->create_subscription<Heartbeat>(heartbeat_topic, rclcpp::QoS(10).best_effort(),
        [&, role](Heartbeat::ConstSharedPtr msg) { latest[role] = *msg; ++received[role]; });
    diagnostic_subs[role] = node->create_subscription<Diagnostic>(state_topic, rclcpp::QoS(10).reliable(),
        [&, role](Diagnostic::ConstSharedPtr msg) { diagnostics[role] = *msg; ++diagnostic_received[role]; });
    functional_safety::HeartbeatConfig config;
    config.node_id = ids[role]; config.topic = heartbeat_topic;
    heartbeats[role] = std::make_unique<functional_safety::HeartbeatPublisher>(*node, config);
    heartbeats[role]->Stop();
    monitors[role] = std::make_unique<dtc::Monitor>(dtc::MonitorConfig{}, At(0));
    publishers[role] = node->create_publisher<Diagnostic>(state_topic, rclcpp::QoS(10).reliable());
  }
  auto check = [&](int ms) {
    rclcpp::spin_some(node);
    const auto before = received;
    const auto diagnostic_before = diagnostic_received;
    const auto sequence_before = sequences;
    const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool matched = false;
    do {
      for (size_t role = 0; role < 4; ++role) {
        // This is the same publication function called by DriverNode's timer.
        dtc::PublishDiagnosticSnapshot(*heartbeats[role], monitors[role]->Evaluate(At(ms)), role,
            "livox-dtc-v3-bounded-sync", ++sequences[role], *publishers[role]);
        heartbeats[role]->PublishNow();
      }
      rclcpp::spin_some(node);
      matched = true;
      for (size_t role = 0; role < 4; ++role)
        matched &= received[role] > before[role] && diagnostic_received[role] > diagnostic_before[role] &&
            diagnostics[role].comm_header.seq > sequence_before[role] &&
            latest[role].active_fault == expected[role];
      if (!matched) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (!matched && std::chrono::steady_clock::now() < end);
    Require(matched, "four driver heartbeats did not deliver their expected DTCs");
    for (size_t role = 0; role < 4; ++role) {
      Require(latest[role].node_id == ids[role] && latest[role].boot_id == heartbeats[role]->BootId(), "wrong heartbeat identity");
      Require(latest[role].work_seq == 0, "diagnostics/heartbeat must not claim pointcloud work");
      Require(diagnostics[role].comm_header.source_id == ids[role] && diagnostics[role].role == role &&
          diagnostics[role].boot_id == latest[role].boot_id && diagnostics[role].observations.size() == 11,
          "source diagnostics must retain the eleven observations and shared driver identity");
    }
  };
  auto hms = [&](size_t role, int ms, std::initializer_list<uint32_t> codes) {
    std::array<uint32_t, 8> values{};
    std::copy(codes.begin(), codes.end(), values.begin());
    monitors[role]->ObserveHms(values, At(ms));
    monitors[role]->ObserveWorkState(1, At(ms));
    monitors[role]->ObservePacket(At(ms));
    monitors[role]->ObserveCloud(1000 + ms, 10, At(ms));
  };
  check(0); // Startup UNKNOWN produces no fabricated fault.
  int ms = 10;
  const std::array<uint32_t, 9> native{{0x01040002, 0x04060002, 0x02100002, 0x01020002,
      0x01160002, 0x01050002, 0x01130002, 0x01170002, 0x01180002}};
  const std::array<uint32_t, 9> offsets{{0, 1, 3, 4, 7, 8, 9, 10, 11}};
  // Isolate each device while exercising all nine native current-fault categories.
  for (size_t category = 0; category < native.size(); ++category) for (size_t role = 0; role < 4; ++role) {
    for (size_t other = 0; other < 4; ++other) {
      hms(other, ms, other == role ? std::initializer_list<uint32_t>{native[category]} : std::initializer_list<uint32_t>{});
      expected[other] = other == role ? std::vector<uint32_t>{bases[role] + offsets[category]} : std::vector<uint32_t>{};
    }
    check(ms++);
  }
  for (size_t role = 0; role < 4; ++role) {
    hms(role, ms, {0x01040002, 0x04060002, 0x04010002, 0x04050002});
    expected[role] = {bases[role], bases[role] + 1};
  }
  check(ms++); // Two current faults; two history entries remain internal.
  for (const auto& state : diagnostics)
    Require(state.observations[9].physical_condition == 2 && state.observations[10].physical_condition == 2,
        "historical HMS events must remain visible in source diagnostics");
  const auto heartbeat_sequence = latest[0].heartbeat_seq;
  check(ms + 2600); // Expired HMS is UNKNOWN; cannot recover active faults.
  Require(latest[0].heartbeat_seq > heartbeat_sequence, "heartbeat sequence stalled during UNKNOWN");
  ms += 2700;
  for (size_t role = 0; role < 4; ++role) {
    hms(role, ms, {0x04060002, 0x04010002, 0x04050002});
    expected[role] = {bases[role] + 1};
  }
  check(ms++); // Only explicitly passing codes recover.
  for (size_t role = 0; role < 4; ++role) {
    // A verified small-offset warning still belongs in the original DTC set.
    monitors[role]->ObserveTimeSync(1, 10000000000ULL, 10000000000ULL, 0, At(ms));
    monitors[role]->ObserveTimeSync(1, 10001000000ULL, 10001000000ULL, 0, At(ms + 1));
    monitors[role]->ObserveTimeSync(1, 10002000000ULL, 10002000000ULL, 0, At(ms + 2));
  }
  check(ms + 2);
  for (const auto& state : diagnostics) Require(state.bounded_time_sync_warning, "expected independent bounded proof");
  ms += 10;
  for (size_t role = 0; role < 4; ++role) {
    hms(role, ms, {0x04010002, 0x04050002}); expected[role].clear();
  }
  check(ms);
  Require(node->count_publishers("/diagnostics/heartbeat/livox_dtc") == 0, "unregistered aggregate heartbeat must not exist");
  functional_safety::HeartbeatConfig restart_config;
  restart_config.node_id = 7; restart_config.topic = "/test/restarted_front";
  functional_safety::HeartbeatPublisher restarted(*node, restart_config);
  restarted.Stop();
  Require(restarted.BootId() != heartbeats[0]->BootId(), "driver restart must change boot ID");
  rclcpp::shutdown();
  std::cout << "driver_dtc_reporting_test passed: four real ROS topics, nine categories, role isolation, history, UNKNOWN, recovery, bounded warning\n";
}
