#ifndef LIVOX_LIDAR_IDENTITY_H_
#define LIVOX_LIDAR_IDENTITY_H_

#include <algorithm>
#include <mutex>
#include <string>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "livox_lidar_def.h"
#include "livox_ros_driver2/msg/lidar_device_info.hpp"

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
class Publisher {
 public:
  using Message = livox_ros_driver2::msg::LidarDeviceInfo;
  Publisher(rclcpp::Node& node, const std::string& role, const std::string& frame)
      : node_(node) {
    if (role != "front" && role != "rear" && role != "left" && role != "right") {
      throw std::invalid_argument("device_info.role must be front/rear/left/right");
    }
    last_.role = role;
    last_.header.frame_id = frame;
    publisher_ = node.create_publisher<Message>("/sensors/livox/" + role + "/device_info",
        rclcpp::QoS(1).reliable().transient_local());
  }

  // Called before SDK startup, from the parsed JSON configuration, never from discovery.
  void Configure(uint32_t handle, const std::string& ip) {
    std::lock_guard<std::mutex> lock(mutex_);
    handle_ = handle;
    configured_ = true;
    observed_ = false;
    last_.ip = ip;
    last_.serial_number.clear();
    last_.device_type = 0;
    last_.identity_valid = false;
    last_.header.stamp = builtin_interfaces::msg::Time();
    publisher_->publish(last_);
  }

  bool Observe(uint32_t handle, const LivoxLidarInfo* info) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_ || !info || handle != handle_ || BoundedString(info->lidar_ip) != last_.ip) {
      return false;
    }
    const auto raw_sn = BoundedString(info->sn);
    const bool valid = ValidSerial(raw_sn);
    const auto sn = valid ? raw_sn : std::string();
    if (!observed_ || last_.serial_number != sn || last_.identity_valid != valid ||
        last_.device_type != info->dev_type) {
      RCLCPP_INFO(node_.get_logger(), "LiDAR identity role=%s ip=%s sn=%s valid=%s type=%u",
          last_.role.c_str(), last_.ip.c_str(), valid ? sn.c_str() : "UNKNOWN",
          valid ? "true" : "false", static_cast<unsigned>(info->dev_type));
    }
    observed_ = true;
    last_.serial_number = sn;
    last_.identity_valid = valid;
    last_.device_type = info->dev_type;
    last_.header.stamp = node_.now();
    // A reconnect/reconfirmation refreshes the timestamp even when SN is unchanged.
    publisher_->publish(last_);
    return true;
  }

 private:
  rclcpp::Node& node_;
  std::mutex mutex_;
  uint32_t handle_ = 0;
  bool configured_ = false;
  bool observed_ = false;
  Message last_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
};
}  // namespace identity
}  // namespace livox_ros
#endif
