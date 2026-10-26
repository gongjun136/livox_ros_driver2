#ifndef LIVOX_LIDAR_IDENTITY_H_
#define LIVOX_LIDAR_IDENTITY_H_

#include <algorithm>
#include <mutex>
#include <string>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "livox_lidar_def.h"

namespace livox_ros {
namespace identity {

// The SDK's fixed-width arrays are not assumed to have a trailing NUL.
template <size_t N>
std::string BoundedString(const char (&value)[N]) {
  return std::string(value, std::find(value, value + N, '\0'));
}

inline bool ValidSerial(const std::string& sn) {
  return !sn.empty() && std::all_of(sn.begin(), sn.end(), [](unsigned char c) {
    return c >= 33 && c <= 126;
  });
}

// One explicitly configured LiDAR per role. All SDK-thread access is serialized.
class Parameters {
 public:
  Parameters(rclcpp::Node& node, const std::string& role, const std::string& frame)
      : node_(node), role_(role) {
    if (role != "front" && role != "rear" && role != "left" && role != "right") {
      throw std::invalid_argument("device_info.role must be front/rear/left/right");
    }
    if (!node.has_parameter("device_info.role")) Declare("role", role);
    Declare("frame_id", frame);
    Declare("ip", std::string());
    Declare("serial_number", std::string());
    Declare("device_type", int64_t(0));
    Declare("identity_valid", false);
    Declare("confirmed_at_ns", int64_t(0));
    // A read_only descriptor would also block SDK updates. Reject external writes
    // with a callback; only the scoped SDK writer on THIS thread may update them.
    guard_ = node.add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& values) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto& value : values) {
        if (value.get_name().compare(0, 12, "device_info.") == 0 && Writer() != this) {
          result.successful = false;
          result.reason = "device_info parameters are managed by the driver and are externally read-only";
          break;
        }
      }
      return result;
    });
  }

  // Called before SDK startup, from the parsed JSON configuration, never from discovery.
  void Configure(uint32_t handle, const std::string& ip) {
    std::lock_guard<std::mutex> lock(mutex_);
    handle_ = handle;
    configured_ = true;
    observed_ = false;
    ip_ = ip;
    sn_.clear();
    type_ = 0;
    valid_ = false;
    Update(0);
  }

  bool Observe(uint32_t handle, const LivoxLidarInfo* info) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_ || !info || handle != handle_ || BoundedString(info->lidar_ip) != ip_) {
      return false;
    }
    const auto raw_sn = BoundedString(info->sn);
    const bool valid = ValidSerial(raw_sn);
    const auto sn = valid ? raw_sn : std::string();
    if (!observed_ || sn_ != sn || valid_ != valid || type_ != info->dev_type) {
      RCLCPP_INFO(node_.get_logger(), "LiDAR identity role=%s ip=%s sn=%s valid=%s type=%u",
          role_.c_str(), ip_.c_str(), valid ? sn.c_str() : "UNKNOWN",
          valid ? "true" : "false", static_cast<unsigned>(info->dev_type));
    }
    observed_ = true;
    sn_ = sn;
    valid_ = valid;
    type_ = info->dev_type;
    return Update(node_.now().nanoseconds());
  }

 private:
  template <typename T>
  void Declare(const std::string& name, const T& value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = "Last SDK-confirmed identity, not an online status";
    descriptor.additional_constraints = "Externally read-only; updated only by the driver";
    // Ignore YAML/CLI overrides so an operator cannot supply a fabricated identity.
    node_.declare_parameter<T>("device_info." + name, value, descriptor, true);
  }

  static const Parameters*& Writer() {
    static thread_local const Parameters* writer = nullptr;
    return writer;
  }

  bool Update(int64_t stamp) {
    struct Scope {
      const Parameters* previous;
      explicit Scope(const Parameters* self) : previous(Writer()) { Writer() = self; }
      ~Scope() { Writer() = previous; }
    } scope(this);
    // A single standard GetParameters request sees one atomic identity snapshot.
    const auto result = node_.set_parameters_atomically({
        rclcpp::Parameter("device_info.ip", ip_),
        rclcpp::Parameter("device_info.serial_number", sn_),
        rclcpp::Parameter("device_info.device_type", int64_t(type_)),
        rclcpp::Parameter("device_info.identity_valid", valid_),
        rclcpp::Parameter("device_info.confirmed_at_ns", stamp)});
    if (!result.successful) RCLCPP_ERROR(node_.get_logger(), "Identity parameter update rejected: %s", result.reason.c_str());
    return result.successful;
  }

  rclcpp::Node& node_;
  const std::string role_;
  std::string ip_, sn_;
  uint8_t type_ = 0;
  bool valid_ = false;
  std::mutex mutex_;
  uint32_t handle_ = 0;
  bool configured_ = false;
  bool observed_ = false;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr guard_;
};
}  // namespace identity
}  // namespace livox_ros
#endif
