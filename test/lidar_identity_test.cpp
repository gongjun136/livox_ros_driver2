#include "lidar_identity.h"
#include <cstring>
#include <iostream>
#include <thread>

using livox_ros::identity::Parameters;
namespace {
void Require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
LivoxLidarInfo Sample() {
  LivoxLidarInfo info{};
  std::strcpy(info.lidar_ip, "192.168.3.184");
  info.dev_type = 9;
  std::memcpy(info.sn, "1234567890ABCDEF", 16);
  return info;
}
}
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  const bool serve = argc > 1 && std::string(argv[1]) == "--serve";
  rclcpp::NodeOptions options;
  options.parameter_overrides({rclcpp::Parameter("device_info.serial_number", "FORGED"),
      rclcpp::Parameter("device_info.ip", "192.0.2.99"),
      rclcpp::Parameter("device_info.identity_valid", true)});
  auto node = std::make_shared<rclcpp::Node>("livox_front_publisher", options);
  try {
    Parameters identity(*node, "front", "test_frame");
    auto info = Sample();
    Require(!identity.Observe(42, &info), "unconfigured discovery accepted");
    identity.Configure(42, "192.168.3.184");
    Require(node->get_parameter("device_info.serial_number").as_string().empty() &&
            !node->get_parameter("device_info.identity_valid").as_bool() &&
            node->get_parameter("device_info.confirmed_at_ns").as_int() == 0,
            "startup override fabricated an identity");
    Require(!identity.Observe(99, &info), "other handle accepted");
    Require(!identity.Observe(42, nullptr), "null info accepted");
    std::strcpy(info.lidar_ip, "192.168.1.108");
    Require(!identity.Observe(42, &info), "other IP accepted");
    info = Sample();
    Require(identity.Observe(42, &info), "configured device rejected");
    Require(node->get_parameter("device_info.serial_number").as_string() == "1234567890ABCDEF",
            "fixed-width serial was overread");
    if (serve) {
      // Real production parameter implementation with explicitly synthetic input.
      rclcpp::spin(node);
      return 0;
    }
    const auto before = node->get_parameter("device_info.confirmed_at_ns").as_int();
    Require(identity.Observe(42, &info), "reconfirmation rejected");
    Require(node->get_parameter("device_info.confirmed_at_ns").as_int() > before,
            "reconfirmation time not refreshed");
    for (const auto& name : {"serial_number", "ip", "role", "frame_id"}) {
      Require(!node->set_parameter(rclcpp::Parameter(std::string("device_info.") + name, "FORGED")).successful,
              "external string write accepted");
    }
    Require(!node->set_parameter(rclcpp::Parameter("device_info.identity_valid", false)).successful,
            "external validity write accepted");
    Require(!node->set_parameter(rclcpp::Parameter("device_info.confirmed_at_ns", int64_t(7))).successful,
            "external time write accepted");
    node->declare_parameter("unrelated", 1);
    Require(node->set_parameter(rclcpp::Parameter("unrelated", 2)).successful,
            "unrelated parameter blocked");
    Require(!node->set_parameters_atomically({rclcpp::Parameter("unrelated", 3),
        rclcpp::Parameter("device_info.serial_number", "FORGED")}).successful,
        "mixed atomic update accepted");
    Require(node->get_parameter("unrelated").as_int() == 2, "mixed batch partially changed state");
    std::memset(info.sn, 0, sizeof(info.sn));
    std::strcpy(info.sn, "REPLACEMENT");
    Require(identity.Observe(42, &info), "SDK could not replace identity");
    Require(node->get_parameter("device_info.serial_number").as_string() == "REPLACEMENT", "replacement missing");
    info.sn[0] = '\x01';
    identity.Observe(42, &info);
    Require(!node->get_parameter("device_info.identity_valid").as_bool() &&
            node->get_parameter("device_info.serial_number").as_string().empty(),
            "invalid SN retained old identity");
    std::memset(info.sn, 0, sizeof(info.sn));
    identity.Observe(42, &info);
    Require(!node->get_parameter("device_info.identity_valid").as_bool(), "empty SN became valid");
    auto rear_node = std::make_shared<rclcpp::Node>("livox_rear_publisher");
    Parameters rear(*rear_node, "rear", "rear_frame");
    rear.Configure(43, "192.168.4.143");
    std::strcpy(info.lidar_ip, "192.168.4.143");
    std::strcpy(info.sn, "REAR_SN");
    Require(!identity.Observe(43, &info), "rear leaked into front");
    Require(rear.Observe(43, &info), "rear rejected");
    Require(rear_node->get_parameter("device_info.serial_number").as_string() == "REAR_SN", "rear SN missing");
    // While SDK writes, another thread must never inherit the internal write permission.
    std::atomic<bool> running{true};
    std::thread writer([&]() { while (running) rear.Observe(43, &info); });
    bool rejected = true;
    for (int i = 0; i < 50; ++i) {
      rejected &= !rear_node->set_parameter(rclcpp::Parameter("device_info.serial_number", "FORGED")).successful;
    }
    running = false;
    writer.join();
    Require(rejected, "SDK write permission leaked across threads");
    for (const auto& topic : node->get_topic_names_and_types()) {
      Require(topic.first.find("/device_info") == std::string::npos, "retired identity topic exists");
    }
    std::cout << "Parameter identity, bounds, override protection, atomic writes and thread isolation passed\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
