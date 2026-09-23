# Livox PointCloud2 Zstd 无损压缩

`pointcloud_zstd_compressor` 和 `pointcloud_zstd_decompressor` 属于 `livox_ros_driver2_core`，只依赖 ROS 2、外部 `livox_ros_driver2` 消息包、PointCloud2 和系统 Zstd，不依赖相机 SDK 或 `shm_msgs`。

## 依赖和编译

```bash
sudo apt install libzstd-dev
source /opt/ros/humble/setup.bash

cd /absolute/path/to/livox_sdk
MAKEFLAGS=-j4 colcon build --executor sequential \
  --packages-up-to livox_ros_driver2_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble
source install/setup.bash
```

## 压缩

```bash
ros2 run livox_ros_driver2_core pointcloud_zstd_compressor --ros-args \
  -p compression_level:=1
```

默认映射：

```text
/livox/lidar_192_168_3_184 -> /livox/lidar_192_168_3_184/zstd
/livox/lidar_192_168_1_108 -> /livox/lidar_192_168_1_108/zstd
/livox/lidar_192_168_2_133 -> /livox/lidar_192_168_2_133/zstd
/livox/lidar_192_168_4_143 -> /livox/lidar_192_168_4_143/zstd
```

只压缩 `PointCloud2.data`；header、fields、height、width、point_step、row_step、大小端和 is_dense 均原样保留。

## 解压回放

```bash
ros2 run livox_ros_driver2_core pointcloud_zstd_decompressor
```

默认发布到带 `/decompressed` 后缀的 topic，避免压缩和解压同时运行时形成消息循环。可用 `input_topics`、`output_topics` 参数修改映射。
