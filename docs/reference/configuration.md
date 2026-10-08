@page livox_configuration 网络、Launch 参数与消息契约

# 主机与设备配置

`master_lidar.launch.py` → 三个 `lidar_*.launch.py` → 各自安装目录中的 `MID360s_config_*.json`。这些 Launch 在 Python 中赋值并传入 ROS 参数，未声明可用命令行覆盖的 Launch 参数；调整时修改对应文件并重编。

| 配置文件 | host_ip | lidar_ip |
|---|---|---|
| `MID360s_config_front.json` | 192.168.3.100 | 192.168.3.184 |
| `MID360s_config_left.json` | 192.168.1.100 | 192.168.1.108 |
| `MID360s_config_right.json` | 192.168.2.100 | 192.168.2.133 |
| `MID360s_config_rear.json`（独立后雷达） | 192.168.4.101 | 192.168.4.143 |

前三路当前雷达侧/主机侧端口分别为：

| 通道 | 雷达端口 | 主机端口 |
|---|---:|---:|
| 控制 | 56100 | 56101 |
| 状态推送 | 56200 | 56201 |
| 点云 | 56300 | 56301 |
| IMU | 56400 | 56401 |
| 日志配置 | 56500 | 56501 |

三路使用不同的本机 IP，所以可以复用端口号。后雷达主机侧使用 56104–56504，不能直接套用前三路的抓包过滤目标端口。

JSON `lidar_configs` 中 `pcl_data_type=1` 表示设备的 32 位笛卡尔原始点格式，`pattern_mode=0` 为非重复扫描。它们与 ROS 的 `xfer_format` 是不同层的选择。当前安装姿态配置均为零；定位使用的传感器外参与业务坐标变换由定位配置维护。

# 当前三个单雷达 Launch 的发布参数

| 参数 | 值 | 作用 |
|---|---|---|
| `xfer_format` | 0 | 生成 PointCloud2；1 为共享 Livox CustomMsg |
| `multi_topic` | 1 | 按雷达 IP 分开 Topic |
| `data_src` | 0 | 实时雷达数据源 |
| `publish_freq` | 10.0 | 点云组帧发布频率，Hz |
| `output_data_type` | 0 | ROS Topic 输出 |
| `frame_id` | `livox_frame` | 消息 Header frame |
| `user_config_path` | 对应 JSON | SDK 与驱动设备配置 |

源码入口为 `src/livox_ros_driver2.cpp` 与 @ref livox_ros::Lddc 。旧 LVX、ROS 1 或 PCL 格式注释不代表当前 ROS 2 发布基线支持这些运行方式。

# 消息与时间

- 点云使用 `sensor_msgs/msg/PointCloud2`，字段为 `x/y/z/intensity/tag/line/timestamp`；位置单位 m，逐点 `timestamp` 在此实现中为绝对 ns 的 double。
- IMU 使用 `sensor_msgs/msg/Imu`。`Lddc::InitImuMsg` 保留设备时间戳，角速度来自设备数据，线加速度通过重力系数转换为 m/s²。
- Header 与点时间来自 SDK 数据包。同步包使用设备给出的 PTP/GPS 时间；未同步包在 `PubHandler::GetEthPacketTimestamp` 中使用主机高精度时钟。
- `frame_id` 是标签；不能仅凭它推断三台设备的坐标已对齐。点云 extrinsic 开关与定位外参需分别对照源码和配置。
- CAN 与 IMU 必须比较实际 Header 时域。不要根据历史 37 s 偏移在新驱动或算法中固定加减秒数。

代码依据：`src/lddc.cpp` 的 `InitPointcloud2MsgHeader`、`InitPointcloud2Msg`、`InitImuMsg`；`src/comm/pub_handler.cpp` 的 `GetEthPacketTimestamp`。
