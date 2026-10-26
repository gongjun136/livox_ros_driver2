#include "lidar_identity.h"
#include <cstring>
#include <iostream>
#include <thread>

using livox_ros::identity::Publisher;
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
template <typename Predicate>
void Until(rclcpp::Node::SharedPtr node, Predicate predicate) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(4);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  Require(predicate(), "DDS delivery timeout");
}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("lidar_identity_test");
  try {
    Publisher publisher(*node, "front", "test_frame");
    LivoxLidarInfo info{};
    std::strcpy(info.lidar_ip, "192.168.3.184");
    info.dev_type = 9;
    std::memcpy(info.sn, "1234567890ABCDEF", 16); // No NUL; never overread into IP.
    Require(!publisher.Observe(42, &info), "unconfigured discovery accepted");
    publisher.Configure(42, "192.168.3.184");
    Publisher::Message latest;
    size_t count = 0;
    auto subscription = node->create_subscription<Publisher::Message>(
        "/sensors/livox/front/device_info", rclcpp::QoS(1).reliable().transient_local(),
        [&](Publisher::Message::ConstSharedPtr msg) { latest = *msg; ++count; });
    Until(node, [&]() { return count > 0; });
    Require(!latest.identity_valid && latest.header.stamp.sec == 0 && latest.serial_number.empty(),
            "startup must be explicitly unknown");
    Require(!publisher.Observe(99, &info), "other device handle accepted");
    Require(!publisher.Observe(42, nullptr), "null info accepted");
    std::strcpy(info.lidar_ip, "192.168.1.108");
    Require(!publisher.Observe(42, &info), "other device IP accepted");
    std::strcpy(info.lidar_ip, "192.168.3.184");
    Require(publisher.Observe(42, &info), "configured device rejected");
    Until(node, [&]() { return latest.identity_valid; });
    Require(latest.serial_number == "1234567890ABCDEF" && latest.role == "front" &&
            latest.header.frame_id == "test_frame" && latest.device_type == 9, "identity mismatch");
    const auto old_time = rclcpp::Time(latest.header.stamp).nanoseconds();
    count = 0;
    Require(publisher.Observe(42, &info), "reconfirmation rejected");
    Until(node, [&]() { return count > 0; });
    Require(rclcpp::Time(latest.header.stamp).nanoseconds() > old_time, "reconfirmation did not refresh time");
    // Subscriber starts AFTER publication; durable QoS must deliver the cached identity.
    size_t late_count = 0;
    Publisher::Message late;
    auto late_sub = node->create_subscription<Publisher::Message>(
        "/sensors/livox/front/device_info", rclcpp::QoS(1).reliable().transient_local(),
        [&](Publisher::Message::ConstSharedPtr msg) { late = *msg; ++late_count; });
    Until(node, [&]() { return late_count > 0; });
    Require(late.serial_number == "1234567890ABCDEF", "late subscriber lost cached identity");
    std::memset(info.sn, 0, sizeof(info.sn));
    std::strcpy(info.sn, "REPLACEMENT");
    publisher.Observe(42, &info);
    Until(node, [&]() { return latest.serial_number == "REPLACEMENT"; });
    info.sn[0] = '\x01';
    publisher.Observe(42, &info);
    Until(node, [&]() { return !latest.identity_valid; });
    Require(latest.serial_number.empty(), "invalid identity retained an old SN");
    std::memset(info.sn, 0, sizeof(info.sn));
    count = 0;
    publisher.Observe(42, &info);
    Until(node, [&]() { return count > 0; });
    Require(!latest.identity_valid && latest.serial_number.empty(), "empty identity became valid");
    // Distinct role topics must not overwrite one another.
    Publisher rear(*node, "rear", "rear_frame");
    rear.Configure(43, "192.168.4.143");
    std::strcpy(info.lidar_ip, "192.168.4.143");
    std::strcpy(info.sn, "REAR_SN");
    Require(!publisher.Observe(43, &info), "rear leaked into front identity");
    Require(rear.Observe(43, &info), "rear identity rejected");
    bool bad_role = false;
    try { Publisher bad(*node, "unknown", ""); } catch (const std::invalid_argument&) { bad_role = true; }
    Require(bad_role, "invalid role accepted");
    std::cout << "Identity binding, bounds, invalidation, replacement, reconfirmation and late DDS delivery passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
