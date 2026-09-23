# 六路相机和四路 Livox 单 MCAP 录制

压缩实现保持在各自驱动内：

```text
camera_sdk/shm_msgs       -> 六路 Image6m H.265
livox_ros_driver2_core    -> 四路 PointCloud2 Zstd
record_compressed_sensors -> 一个 ros2 bag record / 一个 MCAP
```

两个驱动可独立编译。统一录制脚本只在运行时要求两个包都已安装并 source。

## 运行

先启动相机和雷达驱动，然后：

```bash
source /opt/ros/humble/setup.bash
source /absolute/path/to/camera_sdk/ros2_shm/install/setup.bash
source /absolute/path/to/livox_sdk/install/setup.bash

export VIDEOENC_LIBRARY=/absolute/path/to/camera_sdk/cam_geac/lib/libvideoenc.so
export CAMERA_BITRATE_MBPS=15
export CAMERA_GOP=30
export ZSTD_LEVEL=1

ros2 run livox_ros_driver2_core record_compressed_sensors_mcap.sh \
  600 /data/rosbag/compressed_sensors_test
```

脚本依次启动：

```text
ros2 run shm_msgs shm_image6m_h265_encoder_6ch
ros2 run livox_ros_driver2_core pointcloud_zstd_compressor
ros2 bag record ... 10 compressed topics
```

按 `Ctrl+C` 时会先让 recorder 刷盘，再停止两个压缩节点。不执行分路录制或后期合并。

## 验证

建议先录制 30 秒：

```bash
ros2 bag info /data/rosbag/compressed_sensors_test
```

每路 30 Hz 相机应约有 900 条 H.265 消息；压缩节点日志中的 `rejected`、`input_failures`、`invalid_callbacks` 应为 0。相机 H.265 导出和帧序号检查方法见相机包中的 `docs/CAMERA_H265_COMPRESSION.md`。
