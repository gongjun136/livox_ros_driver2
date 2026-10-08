@page livox_architecture 系统架构与代码入口

# 仓库与包的职责

| 工作区源码 | ROS/CMake 包 | 职责 |
|---|---|---|
| `Livox-SDK2/` | `livox_sdk2` | 设备发现、控制、UDP 通信与原始数据回调 |
| `livox_ros_driver2/` | `livox_ros_driver2_core` | ROS 节点、点云组帧、IMU 发布、心跳和压缩工具 |
| `common_msgs/livox_ros_driver/` | `livox_ros_driver2` | 共享 `CustomMsg`、`CustomPoint`、`CompressedPointCloud2` 接口 |
| `common_msgs/diagnostic_monitor_interfaces/` | `diagnostic_monitor_interfaces` | 共享 `NodeHeartbeat` 接口 |
| `lightning-lm/` | `lightning_lm` | 订阅原始点云与主 IMU，执行建图或定位 |

消息定义只有一个来源。驱动实现包与消息包名字不同；启动使用 `livox_ros_driver2_core`，自定义消息类型使用 `livox_ros_driver2/msg/*`。

# 主路径

```text
Launch + JSON
  -> DriverNode / LdsLidar
  -> Livox SDK2 发现、配置设备和接收 UDP
  -> PubHandler 的 IMU 分支 / 点云队列与组帧
  -> Lds 的数据队列与信号量
  -> Lddc 发布 ROS 消息
  -> 原始数据订阅者（定位或录包）
```

# 代码入口

| 问题 | API / 文件 |
|---|---|
| 节点参数、发布线程和数据工作计数 | @ref livox_ros::DriverNode ；`src/livox_ros_driver2.cpp` |
| 初始化 SDK、配置与回调注册 | @ref livox_ros::LdsLidar ；`src/lds_lidar.cpp` |
| 设备控制与进入 Normal 模式 | `src/call_back/livox_lidar_callback.cpp` |
| SDK 数据分支、点云组帧与时间戳转换 | @ref livox_ros::PubHandler ；`src/comm/pub_handler.cpp` |
| PointCloud2、CustomMsg 与 IMU 发布 | @ref livox_ros::Lddc ；`src/lddc.cpp` |
| 心跳线程与序列计数 | @ref livox_ros::functional_safety::HeartbeatPublisher |
| 节点退出与线程等待 | `src/driver_node.cpp`；`DriverNode::~DriverNode` |

统一工作区不会把三台雷达合为一个驱动进程。`master_lidar.launch.py` 包含三个单雷达 Launch，各自启动一个 `livox_ros_driver2_node`。
