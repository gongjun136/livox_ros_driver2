@page livox_compressed_recording 六路相机和四路 Livox 单 MCAP 录制

压缩实现保持在各自驱动内：

```text
camera_sdk/shm_msgs       -> 六路 Image6m H.265
livox_ros_driver2_core    -> 四路 PointCloud2 Zstd
record_compressed_sensors -> 一个 ros2 bag record / 一个 MCAP
```

两个驱动可独立编译。本页描述本仓库包内的兼容脚本；它已打印弃用提示。SDK 层也有同名入口，职责不同，执行前先确认实际文件与安装版本。

# 同名脚本会替我启动哪些节点？ {#livox_compressed_entry_reading}

包内脚本开头已经说明其状态：

@snippet{lineno} record_compressed_sensors_mcap.sh livox-record-compat-entry

它要求相机 Topic 已存在，仅创建雷达压缩子进程：

@snippet{lineno} record_compressed_sensors_mcap.sh livox-record-compressor-start

| 入口 | 当前源码行为 | 调用前需要准备 |
|---|---|---|
| 本包 `ros2 run livox_ros_driver2_core record_compressed_sensors_mcap.sh` | 弃用兼容入口；启动雷达压缩器，等待六相机+四雷达共十个压缩 Topic，然后启动一个 recorder | 相机 H.265 发布端、原始雷达点云、消息包和 QoS 文件 |
| SDK 层 `sdk/scripts/record_compressed_sensors_mcap.sh` | 独立于本仓库；当前本地版本只录制已有 Topic，另外检查类型和单一发布者 | 相机和雷达压缩发布端都需提前启动，命令与包名以其匹配版本为准 |

<b>例子：</b>只启动原始相机图像和雷达，再调用包内兼容入口，不会自动产生六路 H.265。脚本会等待并在缺少所需 Topic 时失败。即使相机 H.265 已有数据而雷达原始点云没有数据，压缩器构造时仍可能创建全部输出 Topic；因此 Topic 齐全并不能证明十路都收到消息。

<b>边界与停止点：</b>本页代码片段只随本仓库生成，SDK 层脚本需单独核验版本；它不是这里的 `@example` 的另一别名。源码没有自适应三雷达列表，也不录 IMU 或 CGI；需要融合数据集时必须另行补齐相应观测。完整脚本：@ref record_compressed_sensors_mcap.sh "包内兼容录制源码"。

# 运行

以下命令专用于包内兼容入口。先启动雷达及相机的 H.265 发布模式，并 source 已部署的对应安装树；不要再同时启动第二个雷达压缩器：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/camera_sdk/ros2_shm/install/setup.bash
source /absolute/path/to/livox_sdk/install/setup.bash

export ZSTD_LEVEL=1

ros2 run livox_ros_driver2_core record_compressed_sensors_mcap.sh \
  600 /data/rosbag/compressed_sensors_test
```

本兼容脚本启动：

```text
ros2 run livox_ros_driver2_core pointcloud_zstd_compressor
ros2 bag record ... 10 compressed topics
```

相机编码参数由相机启动入口配置，本脚本不读取 `CAMERA_BITRATE_MBPS` 或 `CAMERA_GOP`。按 `Ctrl+C` 时，请求停止本脚本拥有的 recorder 与雷达压缩器；不会停止外部相机发布端。不执行分路录制或后期合并。一个 recorder/输出目录不代表一份文件在所有存储设置下永不分卷。

# 正常收尾能证明数据完整吗？ {#livox_compressed_flush_reading}

@snippet{lineno} record_compressed_sensors_mcap.sh livox-record-recorder-cleanup

<b>关键语句：</b>只在子进程仍存在时发送 SIGINT，然后 `wait ... || true`。等待没有超时，退出状态也不用于证明刷盘成功。录制循环检查 recorder 存活，但不会持续检查压缩器是否仍存活；压缩器在录制中退出可能留下仍在运行、数据不再增长的 recorder。

<b>例子：</b>录制 30 秒期间雷达压缩器在第 10 秒退出，相机继续输出，录制目录仍可能产生可读文件。结束提示和 MCAP 可打开都不足以确认雷达覆盖了全程。停止链路、压缩字节正确性和采集完整性要分别检查。

<b>停止点：</b>读到 PID 判断、SIGINT 和等待状态处理即可。保留退出日志，核对每个预期 Topic 的首尾时间、数量、最大间隔、重复时间戳，并抽样解码；无真实数据时不填写“录制验收通过”。

# 验证

建议先录制 30 秒：

```bash
ros2 bag info /data/rosbag/compressed_sensors_test
```

如果相机实际稳定以 30 Hz 发布，30 秒约 900 条消息可作为数量量级核对，启动发现延迟和实际覆盖时长仍需计入。雷达 Zstd 日志使用 `frames`、`failures`、`ratio`；它们与 recorder 接收数量不是同一个统计量。无损字节核对见 @ref livox_zstd_decode_reading 。相机 H.265 的帧序号与解码校验按对应相机版本的 `docs/CAMERA_H265_COMPRESSION.md` 执行。
