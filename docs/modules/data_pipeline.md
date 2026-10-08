@page livox_data_pipeline SDK 接收、组帧与 ROS 发布

# 初始化与控制

@ref livox_ros::DriverNode 在 `src/livox_ros_driver2.cpp` 读取 ROS 参数，建立 @ref livox_ros::Lddc 与 @ref livox_ros::LdsLidar ，调用 SDK 初始化并注册设备/数据回调。

`LivoxLidarCallback::LidarInfoChangeCallback` 配置点类型、扫描模式与安装姿态，再请求 Normal 工作模式和 IMU 推送。控制命令成功只说明通信与配置阶段通过，不能证明点云数据正在输出。

# IMU 分支

`PubHandler::OnLivoxLidarPointCloudCallback` 先按数据类型分流。IMU 包直接构造 `ImuData` 并调用 IMU 回调，进入 IMU 队列，再由独立发布线程交给 `Lddc::DistributeImuData`。

# 点云分支

点云包携带设备 handle、点格式、点数、点间隔和首点时间，进入原始包队列。`PubHandler::RawDataProcess` 转换原始点；`CheckTimer` 根据配置发布间隔组帧，再交给 Lds 点云队列和 `Lddc::DistributePointCloudData`。

已同步时间的组帧路径检查最近时间与发布间隔的边界、帧时间差和点数。未同步时间的路径使用主机计时。修改这段逻辑时同时检查时间回退、包跳变、跨帧缓存和线程退出，不要只改变 Topic 发布频率。

# 发布与观察

`Lddc::PublishPointcloud2` 从队列取帧，构造消息并通过对应 Publisher 发布。Publisher 按需创建，因此正常日志中的 `livox/lidar_... publish use PointCloud2 format` 是驱动进入点云发布路径的证据。

只有 `livox/imu_... publish use imu format` 时，IMU 路径已经进入，点云路径仍需独立检查。先抓取源端口 56300 的包，再区分雷达未输出、Socket 未收到、组帧未完成与 DDS 订阅问题，详见 @ref livox_troubleshooting 。

# 线程与退出边界

节点分别维护点云和 IMU 发布线程，SDK 与 PubHandler 另有接收/处理线程。`DriverNode::~DriverNode` 停止心跳、请求数据源退出、发出退出信号并 join 发布线程；Lddc/Lds/SDK 的后续清理要与所有权一起审查。

2026-10-08 点云正常时仍复现退出 `-11`；没有点云时现场曾因退出等待超时升级到 SIGKILL。上述退出问题仅记录为已观察现象，根因尚未定位，本次文档改动不包含修复。
