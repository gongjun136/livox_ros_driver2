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

#include "functional_safety_heartbeat.h"  // NOLINT(build/include_subdir)

#include <random>
#include <stdexcept>
#include <utility>

namespace livox_ros {
namespace functional_safety {

HeartbeatPublisher::HeartbeatPublisher(rclcpp::Node& node, HeartbeatConfig config)
    : config_(std::move(config)), boot_id_(GenerateBootId()) {
  if (config_.node_id == 0) {
    throw std::invalid_argument("heartbeat node_id must be non-zero");
  }
  if (config_.topic.empty() || config_.topic.front() != '/') {
    throw std::invalid_argument("heartbeat topic must be an absolute ROS topic");
  }
  if (config_.period.count() <= 0 || config_.qos_depth == 0) {
    throw std::invalid_argument("heartbeat period and QoS depth must be positive");
  }

  rclcpp::QoS qos(rclcpp::KeepLast(config_.qos_depth));
  qos.durability_volatile();
  if (config_.reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  publisher_ = node.create_publisher<diagnostic_monitor_interfaces::msg::NodeHeartbeat>(
      config_.topic, qos);
  worker_ = std::thread([this]() { Run(); });
}

HeartbeatPublisher::~HeartbeatPublisher() {
  Stop();
}

void HeartbeatPublisher::SetState(NodeState state) {
  state_.store(static_cast<std::uint8_t>(state), std::memory_order_relaxed);
}

void HeartbeatPublisher::RecordWork() {
  Advance(work_seq_);
}

void HeartbeatPublisher::PublishNow() {
  diagnostic_monitor_interfaces::msg::NodeHeartbeat message;
  message.node_id = config_.node_id;
  message.boot_id = boot_id_;
  message.heartbeat_seq = Advance(heartbeat_seq_);
  message.work_seq = work_seq_.load(std::memory_order_relaxed);
  message.state = state_.load(std::memory_order_relaxed);
  publisher_->publish(message);
}

void HeartbeatPublisher::Stop() {
  const bool already_stopped = stop_requested_.exchange(true);
  if (already_stopped) return;
  wait_condition_.notify_all();
  if (worker_.joinable()) worker_.join();
}

std::uint64_t HeartbeatPublisher::GenerateBootId() {
  std::random_device random;
  std::uint64_t boot_id = 0;
  while (boot_id == 0) {
    boot_id = (static_cast<std::uint64_t>(random()) << 32U) ^
              static_cast<std::uint64_t>(random());
  }
  return boot_id;
}

std::uint32_t HeartbeatPublisher::Advance(std::atomic<std::uint32_t>& sequence) {
  return sequence.fetch_add(1, std::memory_order_relaxed) + std::uint32_t{1};
}

void HeartbeatPublisher::Run() {
  auto deadline = std::chrono::steady_clock::now();
  std::unique_lock<std::mutex> lock(wait_mutex_);
  while (!stop_requested_.load(std::memory_order_relaxed)) {
    lock.unlock();
    PublishNow();
    lock.lock();
    deadline += config_.period;
    if (std::chrono::steady_clock::now() > deadline + config_.period) {
      deadline = std::chrono::steady_clock::now() + config_.period;
    }
    wait_condition_.wait_until(lock, deadline, [this]() {
      return stop_requested_.load(std::memory_order_relaxed);
    });
  }
}

}  // namespace functional_safety
}  // namespace livox_ros
