@page livox_configuration 网络、Launch 参数与消息契约

# 雷达 SN 与设备身份参数

SANY 单雷达入口按现有心跳 ID 选择 front/rear/left/right，默认启用身份参数。
自定义入口可设置 `device_info.role` 和 `device_info.enabled`；无已知角色的通用入口默认关闭。
JSON 必须恰好配置一台雷达，SDK 回调的 handle/IP 必须与配置匹配。

| 参数 | 类型 | 语义 |
|---|---|---|
| `device_info.role` | string | front/rear/left/right；启动后禁止外部修改 |
| `device_info.ip` | string | 配置并经回调核对的设备 IP |
| `device_info.serial_number` | string | SDK 读取的 SN；未知或无效时为空 |
| `device_info.device_type` | integer | SDK 设备类型代码；初始为零 |
| `device_info.identity_valid` | bool | SN 是否非空且为可打印非空白 ASCII |
| `device_info.confirmed_at_ns` | integer | 最近一次接受 SDK 回调的 ROS 时间，纳秒；初始为零 |
| `device_info.frame_id` | string | 驱动坐标系 |

实现入口是 `LivoxLidarCallback::LidarInfoChangeCallback` 和 `identity::Parameters`。
最多读取 SDK SN 字段的 16 字节；无效 SN 清空旧值，不以 IP、MAC 或历史值替代。
SDK 回调原子更新身份及确认时间，标准 GetParameters 一次请求可读取一致快照。
外部参数写入由校验回调拒绝；SDK 写权限仅在当前更新线程的作用域中有效。
身份结果字段忽略 YAML/CLI 覆盖，避免把配置值误认为设备实读值；角色和启用项仍可在启动时配置。

不再创建专用设备身份话题，已删除旧 `LidarDeviceInfo` 消息。没有定时发布或周期查询。
ROS 标准 `/parameter_events` 仍可能在 SDK 更新参数时产生事件；首次发现和身份变化保留日志。
参数只是最近确认身份，**不是在线状态**；断流后原值可能保留，在线状态查 `diagnostic_state`。
该功能不根据 SN 停发点云、改变定位或新增 DTC。

在相同 ROS_DOMAIN_ID 和可互通 DDS 网络中执行：

```bash
source install/local_setup.bash
ros2 param get /livox_front_publisher device_info.serial_number
ros2 param get /livox_front_publisher device_info.identity_valid
ros2 run livox_ros_driver2_core collect_lidar_identity.py \
  --timeout 5 --output /tmp/lidar_identity.json
# 主机三路；从机可用 --roles rear：
ros2 run livox_ros_driver2_core collect_lidar_identity.py --roles front left right
# 使用自定义节点名或命名空间：
ros2 run livox_ros_driver2_core collect_lidar_identity.py \
  --roles front --node front=/vehicle/livox_front_publisher
```

清单工具默认请求 `/livox_<role>_publisher/get_parameters`，每角色至多查询一次，不订阅身份话题。
默认读取 `/opt/conf/vehicle_config.yaml` 的 VIN，可用 `--vin` 或 `--vehicle-config` 覆盖；
VIN 不可用时输出 null。无响应显示 `not_received`，有效响应中无 SN 显示 `unknown`，
缺失/类型错误/角色错误参数显示 `invalid_response`，发现重复节点全名显示 `conflict`。
指定角色全部有效时退出 0，否则退出 2。JSON schema_version 为 2，以 node 替代旧 topic 字段，
保留采集时间、确认时间和来源，不修改车辆配置。主从 DDS 不通时分别导出。

升级应使用干净的 build/install 目录：普通增量安装不会自动删除旧生成消息文件。
本版不依赖旧消息。测试覆盖启动覆盖值、SDK 更新、外部写拒绝、并发权限隔离、
SN 边界、错误设备、替换与无效状态；跨进程测试直接查询 C++ 参数实现，
验证远程写入拒绝、清单读取、缺项及重名节点冲突。
跨进程测试采用测试依赖 CycloneDDS、独立域 179，不改变生产 RMW。
合成测试不等于实机验收；现场需在驱动重启窗口核对 SN 与原点云、IMU、诊断链路。

# 主机与设备配置

`master_lidar.launch.py` → 三个 `lidar_*.launch.py` → 各自安装目录中的 `MID360s_config_*.json`。前三路 Launch 在 Python 中赋值并传入 ROS 参数；调整时修改对应文件并重编。

`master_4lidar.launch.py` 包含 `master_lidar.launch.py` 与 `lidar_rear.launch.py`，为后雷达传入 `config_file` 指向 `MID360s_config_rear_master.json`。后雷达 Launch 的 `config_file` 默认仍为 `MID360s_config_rear.json`，因此旧 slave 入口保持原配置。

| 配置文件 | host_ip | lidar_ip |
|---|---|---|
| `MID360s_config_front.json` | 192.168.3.100 | 192.168.3.184 |
| `MID360s_config_left.json` | 192.168.1.100 | 192.168.1.108 |
| `MID360s_config_right.json` | 192.168.2.100 | 192.168.2.133 |
| `MID360s_config_rear.json`（独立后雷达） | 192.168.4.101 | 192.168.4.143 |
| `MID360s_config_rear_master.json`（master 四雷达入口的后雷达） | 192.168.4.100 | 192.168.4.143 |

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

# 前、左、右、后单雷达 Launch 的发布参数

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

# DTC 同步源验证

对外活动 DTC 由四台驱动各自在 `NodeHeartbeat.active_fault` 发布：
前 `/diagnostics/heartbeat/livox_front`（ID 7），后 `livox_rear`（ID 12），
左 `livox_left`（ID 8），右 `livox_right`（ID 9）；后三者均使用相同话题前缀。
每台只携带自身九类当前故障，诊断中心按四台已有 ID 登记对应码。
FAIL 加入，只有新鲜 PASS 清除，UNKNOWN/过期保留；历史码 base+12/base+13 不进入心跳。
完整十一项观测（含历史、证据年龄和设备可用性）仍由各台
`/sensors/livox/{front,rear,left,right}/diagnostic_state` 发布，定位直接消费。
不再启动独立聚合器，不发布 ID 13 心跳或 `/internal/diagnostics/livox_dtc`。
全雷达不可用的组合判定在定位 Safety 内部完成；没有已登记归属的汇总 DTC 不转挂到其他节点。
本次活动缓存不跨进程重启，诊断中心仍须处理 boot_id 变化及启动未知状态。

驱动当前配置版本为 `livox-dtc-v3-bounded-sync`；主、从驱动和定位消费者应同步更新。
升级时停止旧聚合进程，使用新的安装目录，避免残留旧 executable/launch 继续发布 ID 13。
`dtc.state_timeout_ms` 默认 2500 ms，限制状态证据年龄及设备时间到最后同步时刻的间隔；
`dtc.max_sync_offset_ns` 默认 1000000 ns，是确认 PTP 健康后排除未使用 GPS/PPS 源 Warning 的偏差上限。
这两个条件只用于证明当前 PTP 有效，不会放宽点云/点包 500 ms 超时或屏蔽原生 Error/Fatal。
完整字段、同步时刻前进以及当前选用 PTP 缺一不可；重复查询相同的同步时刻不能刷新证明。
GPS/PPS Warning 的排除范围为 0x0404、0x0407–0x0409；PTP 故障、低精度及 PTP/gPTP 冲突仍上报。
0x0401、0x0405 的历史源 DTC 继续可见，由兼容安全接口与当前故障区分处理。

# 消息与时间

- 点云使用 `sensor_msgs/msg/PointCloud2`，字段为 `x/y/z/intensity/tag/line/timestamp`；位置单位 m，逐点 `timestamp` 在此实现中为绝对 ns 的 double。
- IMU 使用 `sensor_msgs/msg/Imu`。`Lddc::InitImuMsg` 保留设备时间戳，角速度来自设备数据，线加速度通过重力系数转换为 m/s²。
- Header 与点时间来自 SDK 数据包。同步包使用设备给出的 PTP/GPS 时间；未同步包在 `PubHandler::GetEthPacketTimestamp` 中使用主机高精度时钟。
- `frame_id` 是标签；不能仅凭它推断三台设备的坐标已对齐。点云 extrinsic 开关与定位外参需分别对照源码和配置。
- CAN 与 IMU 必须比较实际 Header 时域。不要根据历史 37 s 偏移在新驱动或算法中固定加减秒数。

代码依据：`src/lddc.cpp` 的 `InitPointcloud2MsgHeader`、`InitPointcloud2Msg`、`InitImuMsg`；`src/comm/pub_handler.cpp` 的 `GetEthPacketTimestamp`。
