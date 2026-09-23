//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef FUNCTIONAL_SAFETY_HEARTBEAT_H_
#define FUNCTIONAL_SAFETY_HEARTBEAT_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

#include <diagnostic_monitor_interfaces/msg/node_heartbeat.hpp>
#include <rclcpp/rclcpp.hpp>

namespace livox_ros {
namespace functional_safety {

enum class NodeState : std::uint8_t {
  kNotReady = 0,  // SDK/configuration initialization is in progress.
  kIdle = 1,      // Initialized, but no real point cloud has been published.
  kRunning = 2,   // Real point clouds are being published.
  kReserved = 3,  // Reserved; stream loss is represented by a stalled work_seq.
  kFault = 4,     // SDK/configuration initialization failed.
};

struct HeartbeatConfig {
  std::uint16_t node_id = 0;
  std::string topic;
  std::chrono::milliseconds period{100};
  std::size_t qos_depth = 1;
  bool reliable = false;
};

class HeartbeatPublisher final {
 public:
  HeartbeatPublisher(rclcpp::Node& node, HeartbeatConfig config);
  ~HeartbeatPublisher();

  HeartbeatPublisher(const HeartbeatPublisher&) = delete;
  HeartbeatPublisher& operator=(const HeartbeatPublisher&) = delete;

  void SetState(NodeState state);
  void RecordWork();
  void PublishNow();
  void Stop();

  static std::uint64_t GenerateBootId();
  static std::uint32_t Advance(std::atomic<std::uint32_t>& sequence);

 private:
  void Run();

  HeartbeatConfig config_;
  rclcpp::Publisher<diagnostic_monitor_interfaces::msg::NodeHeartbeat>::SharedPtr publisher_;
  const std::uint64_t boot_id_;
  std::atomic<std::uint32_t> heartbeat_seq_{0};
  std::atomic<std::uint32_t> work_seq_{0};
  std::atomic<std::uint8_t> state_{static_cast<std::uint8_t>(NodeState::kNotReady)};
  std::atomic_bool stop_requested_{false};
  std::mutex wait_mutex_;
  std::condition_variable wait_condition_;
  std::thread worker_;
};

}  // namespace functional_safety
}  // namespace livox_ros

#endif  // FUNCTIONAL_SAFETY_HEARTBEAT_H_
