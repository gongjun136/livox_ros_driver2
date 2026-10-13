#include "lidar_dtc_monitor.h"
#include "functional_safety_heartbeat.h"
#include "diagnostic_monitor_interfaces/msg/dtc_observation_array.hpp"
#include "livox_ros_driver2/msg/lidar_diagnostic_state.hpp"
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <stdexcept>

namespace livox_ros {
class LidarDtcAggregator final : public rclcpp::Node {
 public:
  LidarDtcAggregator() : Node("lidar_dtc_aggregator"), start_(dtc::Clock::now()) {
    const auto id = declare_parameter<int>("functional_safety.node_id", 13);
    if (id <= 0 || id > 65535) throw std::invalid_argument("invalid aggregator source ID");
    source_id_ = id;
    functional_safety::HeartbeatConfig config;
    config.node_id = source_id_;
    config.topic = declare_parameter<std::string>("functional_safety.heartbeat_topic", "/diagnostics/heartbeat/livox_dtc");
    heartbeat_ = std::make_unique<functional_safety::HeartbeatPublisher>(*this, config);
    const auto ids = declare_parameter<std::vector<int64_t>>("expected_source_ids", {7,12,8,9});
    if (ids.size() != 4 || std::set<int64_t>(ids.begin(), ids.end()).size() != 4) throw std::invalid_argument("four distinct role IDs required");
    for (size_t i = 0; i < 4; ++i) {
      if (ids[i] <= 0 || ids[i] > 65535 || ids[i] == source_id_) throw std::invalid_argument("invalid/conflicting role ID");
      expected_ids_[i] = ids[i];
    }
    const int timeout = declare_parameter<int>("state_timeout_ms", 500);
    const int hms_ttl = declare_parameter<int>("hms_timeout_ms", 2500);
    const int grace = declare_parameter<int>("startup_grace_ms", 8000);
    if (timeout <= 0 || hms_ttl <= 0 || grace < 0) throw std::invalid_argument("invalid aggregator timeouts");
    timeout_ = std::chrono::milliseconds(timeout);
    hms_ttl_us_ = static_cast<uint64_t>(hms_ttl) * 1000;
    grace_ = std::chrono::milliseconds(grace);
    version_ = declare_parameter<std::string>("config_version", "livox-dtc-v1-bench");
    const auto topic = declare_parameter<std::string>("output_topic", "/diagnostics/source_dtc/livox");
    publisher_ = create_publisher<diagnostic_monitor_interfaces::msg::DtcObservationArray>(topic, rclcpp::QoS(10).reliable().durability_volatile());
    const std::array<std::string,4> names{{"front","rear","left","right"}};
    for (size_t i = 0; i < 4; ++i) {
      subscriptions_[i] = create_subscription<livox_ros_driver2::msg::LidarDiagnosticState>(
          "/sensors/livox/" + names[i] + "/diagnostic_state", rclcpp::QoS(10).reliable().durability_volatile(),
          [this,i](livox_ros_driver2::msg::LidarDiagnosticState::ConstSharedPtr message) { Observe(i,*message); });
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { Publish(); });
  }

 private:
  struct Source {
    livox_ros_driver2::msg::LidarDiagnosticState state;
    dtc::Clock::time_point received;
    bool seen = false;
    std::set<uint64_t> retired_boots;
    std::array<uint32_t, dtc::kCategories> check_seq{};
    std::array<uint8_t, dtc::kCategories> last_condition{};
  };
  void Observe(size_t role, const livox_ros_driver2::msg::LidarDiagnosticState& state) {
    auto& source = sources_[role];
    if (state.role != role || state.comm_header.source_id != expected_ids_[role] || state.boot_id == 0 ||
        state.config_version != version_ || state.usability > dtc::kUnusable || state.observations.size() != dtc::kCategories) return;
    for (size_t i = 0; i < dtc::kCategories; ++i) {
      if (state.observations[i].dtc != dtc::Codes(role)[i] || state.observations[i].physical_condition > dtc::kFail) return;
    }
    const bool new_boot = !source.seen || source.state.boot_id != state.boot_id;
    if (source.retired_boots.count(state.boot_id)) return;
    if (!new_boot && static_cast<int32_t>(state.comm_header.seq - source.state.comm_header.seq) <= 0) return;
    bool changed = new_boot || source.state.usability != state.usability;
    for (size_t i = 0; i < dtc::kCategories; ++i) {
      const auto& current = state.observations[i];
      if (new_boot || current.evaluation_seq != source.state.observations[i].evaluation_seq ||
          current.physical_condition != source.state.observations[i].physical_condition) ++source.check_seq[i];
      changed = changed || new_boot || current.physical_condition != source.last_condition[i];
    }
    if (source.seen && new_boot) source.retired_boots.insert(source.state.boot_id);
    source.state = state;
    source.received = dtc::Clock::now();
    source.seen = true;
    if (changed) Publish();
  }
  void Publish() {
    const auto now = dtc::Clock::now();
    diagnostic_monitor_interfaces::msg::DtcObservationArray result;
    result.comm_header.source_id = source_id_;
    result.comm_header.seq = ++sequence_;
    result.comm_header.stamp_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    result.boot_id = heartbeat_->BootId();
    result.interface_version = 1;
    result.config_version = version_;
    std::array<uint8_t,4> usability{};
    for (size_t role = 0; role < 4; ++role) {
      auto& source = sources_[role];
      const bool fresh = source.seen && now - source.received <= timeout_;
      usability[role] = fresh ? static_cast<dtc::Usability>(source.state.usability) :
          (source.seen || now - start_ >= grace_ ? dtc::kUnusable : dtc::kUnavailableUnknown);
      for (size_t i = 0; i < dtc::kCategories; ++i) {
        diagnostic_monitor_interfaces::msg::DtcObservation observation;
        observation.dtc = dtc::Codes(role)[i];
        observation.evaluation_age_us = std::numeric_limits<uint64_t>::max();
        if (fresh) {
          observation = source.state.observations[i];
          const auto transit_age = dtc::AgeUs(now,source.received,true);
          if (observation.evaluation_age_us <= std::numeric_limits<uint64_t>::max() - transit_age) observation.evaluation_age_us += transit_age;
          if (observation.evaluation_age_us > hms_ttl_us_) observation.physical_condition = dtc::kUnknown;
        }
        if (observation.physical_condition != source.last_condition[i]) ++source.check_seq[i];
        source.last_condition[i] = observation.physical_condition;
        observation.evaluation_seq = source.check_seq[i];
        result.observations.push_back(observation);
      }
    }
    const auto aggregate = dtc::Aggregate(usability);
    for (size_t i = 0; i < 2; ++i) {
      diagnostic_monitor_interfaces::msg::DtcObservation observation;
      observation.dtc = i == 0 ? dtc::kMultipleFailed : dtc::kAllFailed;
      observation.physical_condition = aggregate[i];
      observation.evaluation_seq = sequence_;
      observation.evaluation_age_us = 0;
      result.observations.push_back(observation);
    }
    std::sort(result.observations.begin(),result.observations.end(),[](const auto& a,const auto& b){return a.dtc<b.dtc;});
    publisher_->publish(result);
    heartbeat_->RecordWork();
    heartbeat_->SetState(functional_safety::NodeState::kRunning);
  }
  uint16_t source_id_ = 0;
  uint32_t sequence_ = 0;
  uint64_t hms_ttl_us_ = 0;
  std::string version_;
  std::array<uint16_t,4> expected_ids_{};
  std::array<Source,4> sources_;
  dtc::Clock::time_point start_;
  std::chrono::milliseconds timeout_{500},grace_{8000};
  std::unique_ptr<functional_safety::HeartbeatPublisher> heartbeat_;
  rclcpp::Publisher<diagnostic_monitor_interfaces::msg::DtcObservationArray>::SharedPtr publisher_;
  std::array<rclcpp::Subscription<livox_ros_driver2::msg::LidarDiagnosticState>::SharedPtr,4> subscriptions_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace livox_ros
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<livox_ros::LidarDtcAggregator>()); }
  catch (const std::exception& error) { std::cerr << error.what() << '\n'; rclcpp::shutdown(); return 1; }
  rclcpp::shutdown();
  return 0;
}
