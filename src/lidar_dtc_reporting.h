#pragma once

#include "functional_safety_heartbeat.h"
#include "lidar_dtc_monitor.h"
#include "livox_ros_driver2/msg/lidar_diagnostic_state.hpp"

namespace livox_ros {
namespace dtc {
// Shared by the driver's timer and the hardware-free ROS regression. Only the
// first nine categories describe current faults; the final two are history.
inline void PublishDiagnosticSnapshot(functional_safety::HeartbeatPublisher& heartbeat,
    const Snapshot& state, size_t role, const std::string& version, uint32_t sequence,
    rclcpp::Publisher<livox_ros_driver2::msg::LidarDiagnosticState>& publisher) {
  livox_ros_driver2::msg::LidarDiagnosticState message;
  message.role = role;
  message.comm_header.source_id = heartbeat.NodeId();
  message.comm_header.seq = sequence;
  message.comm_header.stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  message.boot_id = heartbeat.BootId();
  message.config_version = version;
  message.usability = state.usability;
  message.bounded_time_sync_warning = state.bounded_time_sync_warning;
  message.time_sync_offset_ns = state.time_sync_offset_ns;
  message.time_sync_evidence_age_us = state.time_sync_evidence_age_us;
  // [livox-diagnostic-current-vs-history]
  std::vector<uint32_t> failed, passed;
  for (size_t i = 0; i < kCategories; ++i) {
    diagnostic_monitor_interfaces::msg::DtcObservation observation;
    observation.dtc = Codes(role)[i];
    observation.physical_condition = state.conditions[i];
    observation.evaluation_seq = state.hms_seq;
    observation.evaluation_age_us = state.hms_age_us;
    message.observations.push_back(observation);
    if (i >= kCategories - 2) continue;
    if (state.conditions[i] == kFail) failed.push_back(observation.dtc);
    if (state.conditions[i] == kPass) passed.push_back(observation.dtc);
  }
  heartbeat.UpdateFaults(failed, passed);
  publisher.publish(message);
  // [livox-diagnostic-current-vs-history]
}
}  // namespace dtc
}  // namespace livox_ros
