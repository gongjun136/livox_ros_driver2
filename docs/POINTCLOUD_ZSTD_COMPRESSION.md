@page livox_pointcloud_zstd Livox PointCloud2 Zstd 无损压缩

`pointcloud_zstd_compressor` 和 `pointcloud_zstd_decompressor` 属于 `livox_ros_driver2_core`，只依赖 ROS 2、外部 `livox_ros_driver2` 消息包、PointCloud2 和系统 Zstd，不依赖相机 SDK 或 `shm_msgs`。

# 无损压缩具体保留什么？ {#livox_zstd_lossless_reading}

先读消息赋值，不必进入 Zstd 库内部：

@snippet{lineno} compression/pointcloud_zstd_compressor.cpp livox-zstd-layout

<b>关键语句：</b>布局与时间戳逐项复制，`uncompressed_size` 记录原始 `data.size()`；随后只把点数据字节送进压缩函数：

@snippet{lineno} compression/pointcloud_zstd_compressor.cpp livox-zstd-compress

<b>例子：</b>一帧原始 `data` 有 1,000,000 字节，Zstd 返回 400,000 字节，则输出的 `uncompressed_size` 仍是 1,000,000，`data.size()` 为 400,000。点数、字段偏移和 Header 不随压缩比变化；日志字节比只统计 `data`，不含 ROS 消息元数据或 MCAP 索引开销。

<b>边界与停止点：</b>这里的“无损”指成功解码后可还原输入字节，不等于所有扫描都被接收或写盘。`frames` 在 `publish()` 之前递增，`failures` 只计 Zstd 报错，不能证明订阅端收包或 recorder 落盘。若源 `row_step`、字段布局本身错误，复制不会修正它。读到赋值、压缩失败的提前返回和发布即可停止。完整方法：@ref livox_ros_driver2::compression::PointCloudZstdCompressor::compress "compress()"。

# 解码成功能证明点云布局正确吗？ {#livox_zstd_decode_reading}

解码入口先检查格式和声明的分配上限：

@snippet{lineno} compression/pointcloud_zstd_decompressor.cpp livox-zstd-format-limit

再检查 Zstd 返回值与声明字节数是否一致：

@snippet{lineno} compression/pointcloud_zstd_decompressor.cpp livox-zstd-decode-size

<b>例子：</b>声明 1,000 字节而实际解出 900 字节时，该帧被拒绝；声明超过默认 512 MiB 时，在申请输出数据区之前拒绝。但若成功解出 1,000 字节，而 `height * row_step` 声称 1,200 字节，当前方法没有额外的布局一致性检查，不能称为“合法 PointCloud2 已验证”。

<b>边界与停止点：</b>上限约束的是单帧声明长度，不是进程总内存预算；成功解压也不证明源点云没有时间跳变或缺帧。验收时按同一 Header 对齐原始帧与解码帧，逐项比较布局和 `data` 字节/哈希，再分别统计丢帧、重复与时间间隔。默认 `/decompressed` 后缀降低误接回压缩链的机会，自定义映射仍需核对输入输出没有形成闭环。完整方法：@ref livox_ros_driver2::compression::PointCloudZstdDecompressor::decompress "decompress()"。

# 依赖和编译

```bash
sudo apt install libzstd-dev
source /opt/ros/humble/setup.bash

cd /absolute/path/to/livox_sdk
MAKEFLAGS=-j4 colcon build --executor sequential \
  --packages-up-to livox_ros_driver2_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DDISTRO_ROS=humble
source install/setup.bash
```

# 压缩

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

# 解压回放

```bash
ros2 run livox_ros_driver2_core pointcloud_zstd_decompressor
```

默认发布到带 `/decompressed` 后缀的 topic，避免压缩和解压同时运行时形成消息循环。可用 `input_topics`、`output_topics` 参数修改映射。
