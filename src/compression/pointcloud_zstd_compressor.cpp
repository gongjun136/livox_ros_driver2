#include "livox_ros_driver2/msg/compressed_point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <zstd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace livox_ros_driver2
{
namespace compression
{

using CompressedPointCloud2 = livox_ros_driver2::msg::CompressedPointCloud2;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

struct StreamStats
{
  explicit StreamStats(std::string topic_name)
  : topic(std::move(topic_name)) {}

  std::string topic;
  std::atomic<std::uint64_t> frames{0};
  std::atomic<std::uint64_t> failures{0};
  std::atomic<std::uint64_t> input_bytes{0};
  std::atomic<std::uint64_t> output_bytes{0};
};

class PointCloudZstdCompressor : public rclcpp::Node
{
public:
  PointCloudZstdCompressor()
  : Node("pointcloud_zstd_compressor")
  {
    const auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics",
      {"/livox/lidar_192_168_3_184",
        "/livox/lidar_192_168_1_108",
        "/livox/lidar_192_168_2_133",
        "/livox/lidar_192_168_4_143"});
    const auto output_topics = declare_parameter<std::vector<std::string>>(
      "output_topics",
      {"/livox/lidar_192_168_3_184/zstd",
        "/livox/lidar_192_168_1_108/zstd",
        "/livox/lidar_192_168_2_133/zstd",
        "/livox/lidar_192_168_4_143/zstd"});
    compression_level_ = declare_parameter("compression_level", 1);

    if (input_topics.size() != output_topics.size() || input_topics.empty()) {
      throw std::runtime_error(
              "input_topics and output_topics must have equal non-zero length");
    }
    if (compression_level_ < ZSTD_minCLevel() ||
      compression_level_ > ZSTD_maxCLevel())
    {
      throw std::runtime_error("compression_level is outside the supported range");
    }

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();

    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      auto publisher = create_publisher<CompressedPointCloud2>(output_topics[i], qos);
      auto stats = std::make_shared<StreamStats>(input_topics[i]);
      auto subscription = create_subscription<PointCloud2>(
        input_topics[i], qos,
        [this, publisher, stats](const PointCloud2::ConstSharedPtr input) {
          compress(input, publisher, stats);
        }, options);
      subscriptions_.push_back(std::move(subscription));
      stats_.push_back(std::move(stats));
      RCLCPP_INFO(
        get_logger(), "%s -> %s (Zstd level %d, lossless)",
        input_topics[i].c_str(), output_topics[i].c_str(), compression_level_);
    }

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() {log_stats();});
  }

private:
  void compress(
    const PointCloud2::ConstSharedPtr & input,
    const rclcpp::Publisher<CompressedPointCloud2>::SharedPtr & publisher,
    const std::shared_ptr<StreamStats> & stats)
  {
    CompressedPointCloud2 output;
    output.header = input->header;
    output.height = input->height;
    output.width = input->width;
    output.fields = input->fields;
    output.is_bigendian = input->is_bigendian;
    output.point_step = input->point_step;
    output.row_step = input->row_step;
    output.is_dense = input->is_dense;
    output.compression_format = "zstd";
    output.uncompressed_size = input->data.size();
    output.data.resize(ZSTD_compressBound(input->data.size()));

    const std::size_t compressed_size = ZSTD_compress(
      output.data.data(), output.data.size(), input->data.data(), input->data.size(),
      compression_level_);
    if (ZSTD_isError(compressed_size)) {
      ++stats->failures;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s Zstd error: %s",
        stats->topic.c_str(), ZSTD_getErrorName(compressed_size));
      return;
    }
    output.data.resize(compressed_size);

    ++stats->frames;
    stats->input_bytes += input->data.size();
    stats->output_bytes += compressed_size;
    publisher->publish(std::move(output));
  }

  void log_stats() const
  {
    for (const auto & stats : stats_) {
      const auto input_bytes = stats->input_bytes.load();
      const auto output_bytes = stats->output_bytes.load();
      const double ratio = output_bytes == 0 ? 0.0 :
        static_cast<double>(input_bytes) / static_cast<double>(output_bytes);
      RCLCPP_INFO(
        get_logger(), "%s: frames=%lu failures=%lu ratio=%.2fx",
        stats->topic.c_str(), stats->frames.load(), stats->failures.load(), ratio);
    }
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  std::vector<rclcpp::Subscription<PointCloud2>::SharedPtr> subscriptions_;
  std::vector<std::shared_ptr<StreamStats>> stats_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  int compression_level_{1};
};

}  // namespace compression
}  // namespace livox_ros_driver2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<
      livox_ros_driver2::compression::PointCloudZstdCompressor>();
    rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_zstd_compressor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
