@page livox_recording 录包脚本与工作流选择

# 选择采集入口

| 入口 | 数据与依赖 | 边界 |
|---|---|---|
| `record_3lidars_mcap.sh` / 安装别名 `record_lidars_mcap.sh` | 实际发现的完整点云+IMU Topic 对 | 脚本候选含前、左、右、后；名称中的 3 不限制为三台 |
| `record_lidars_cameras_mcap.sh` | 雷达、IMU 与相机采集 | 按脚本配置确认传感器和独立相机工作区 |
| `record_compressed_sensors_mcap.sh` | 四路点云 Zstd + 六路相机 H.265 | 相机包与雷达包均需安装并 source；一个 recorder/MCAP |
| 定位 `run_loc.sh` | 项目正式定位入口 | 当前固定关闭定位 MCAP，不由以上脚本自动启用 |

# 原始雷达采集

```bash
ros2 run livox_ros_driver2_core record_lidars_mcap.sh \
  30 /data/rosbag/livox_raw_test
```

第一个参数为正整数秒，默认 600；第二个为输出目录，默认按日期在 `/data/rosbag` 下创建。脚本先检查 MCAP 插件与安装树的 QoS 文件，最多等待 30 s 发现完整点云+IMU Topic 对；没有任何完整输入时退出。

发现 Topic 对不能证明收到消息。录包前先完成 @ref livox_sany_deployment 的输入检查，之后以 `ros2 bag info` 核对每路实际数量。脚本按路录制再合并，正常中断需等待写盘与合并收尾；完整源码见 @ref livox_source_examples 。

# 压缩工作流

PointCloud2 压缩只处理 `data` 字节，保留 Header 与布局，解压默认输出 `/decompressed` 后缀避免环路。具体映射、参数和消息定义见 @ref livox_pointcloud_zstd 。

六相机四雷达的压缩录制入口见 @ref livox_compressed_recording 。它的容量、CPU 负载与验收口径需单独测量，不能沿用原始三雷达定位录包的估算。
