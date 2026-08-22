#include "livox_ros_driver2/msg/compressed_point_cloud2.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <zstd.h>

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace livox_ros_driver2
{
namespace compression
{

using CompressedPointCloud2 = livox_ros_driver2::msg::CompressedPointCloud2;
using PointCloud2 = sensor_msgs::msg::PointCloud2;

class PointCloudZstdDecompressor : public rclcpp::Node
{
public:
  PointCloudZstdDecompressor()
  : Node("pointcloud_zstd_decompressor")
  {
    const auto input_topics = declare_parameter<std::vector<std::string>>(
      "input_topics",
      {"/livox/lidar_192_168_3_184/zstd",
        "/livox/lidar_192_168_1_108/zstd",
        "/livox/lidar_192_168_2_133/zstd",
        "/livox/lidar_192_168_4_143/zstd"});
    const auto output_topics = declare_parameter<std::vector<std::string>>(
      "output_topics",
      {"/livox/lidar_192_168_3_184/decompressed",
        "/livox/lidar_192_168_1_108/decompressed",
        "/livox/lidar_192_168_2_133/decompressed",
        "/livox/lidar_192_168_4_143/decompressed"});
    max_uncompressed_bytes_ = declare_parameter<std::int64_t>(
      "max_uncompressed_bytes", 512LL * 1024LL * 1024LL);

    if (input_topics.size() != output_topics.size() || input_topics.empty()) {
      throw std::runtime_error(
              "input_topics and output_topics must have equal non-zero length");
    }
    if (max_uncompressed_bytes_ <= 0) {
      throw std::runtime_error("max_uncompressed_bytes must be positive");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(64)).reliable().durability_volatile();
    for (std::size_t i = 0; i < input_topics.size(); ++i) {
      auto publisher = create_publisher<PointCloud2>(output_topics[i], qos);
      auto subscription = create_subscription<CompressedPointCloud2>(
        input_topics[i], qos,
        [this, publisher, topic = input_topics[i]](
          const CompressedPointCloud2::ConstSharedPtr input)
        {
          decompress(input, publisher, topic);
        });
      subscriptions_.push_back(std::move(subscription));
      RCLCPP_INFO(
        get_logger(), "LiDAR decompression: %s -> %s",
        input_topics[i].c_str(), output_topics[i].c_str());
    }
  }

private:
  void decompress(
    const CompressedPointCloud2::ConstSharedPtr & input,
    const rclcpp::Publisher<PointCloud2>::SharedPtr & publisher,
    const std::string & topic)
  {
    if (input->compression_format != "zstd") {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s has unsupported compression format '%s'",
        topic.c_str(), input->compression_format.c_str());
      return;
    }
    if (input->uncompressed_size > static_cast<std::uint64_t>(max_uncompressed_bytes_)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s declares an unsafe uncompressed size: %lu bytes",
        topic.c_str(), input->uncompressed_size);
      return;
    }

    PointCloud2 output;
    output.header = input->header;
    output.height = input->height;
    output.width = input->width;
    output.fields = input->fields;
    output.is_bigendian = input->is_bigendian;
    output.point_step = input->point_step;
    output.row_step = input->row_step;
    output.is_dense = input->is_dense;
    output.data.resize(static_cast<std::size_t>(input->uncompressed_size));

    const std::size_t result = ZSTD_decompress(
      output.data.data(), output.data.size(), input->data.data(), input->data.size());
    if (ZSTD_isError(result)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "%s Zstd error: %s",
        topic.c_str(), ZSTD_getErrorName(result));
      return;
    }
    if (result != output.data.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "%s size mismatch: expected %zu bytes, decoded %zu bytes",
        topic.c_str(), output.data.size(), result);
      return;
    }

    publisher->publish(std::move(output));
  }

  std::vector<rclcpp::Subscription<CompressedPointCloud2>::SharedPtr> subscriptions_;
  std::int64_t max_uncompressed_bytes_{0};
};

}  // namespace compression
}  // namespace livox_ros_driver2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<livox_ros_driver2::compression::PointCloudZstdDecompressor>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pointcloud_zstd_decompressor"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
