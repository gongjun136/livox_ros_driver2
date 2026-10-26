@page livox_data_pipeline SDK 接收、组帧与 ROS 发布

本页按 ROS 2 原始雷达输入、PointCloud2 输出阅读。先完成 @ref livox_launch_layout_reading "角色与配置选择"，再沿下列四个问题核对。例子均为分支推演，不是实机频率或退出测试。

@anchor livox_packet_routes_reading
# 1. SDK 收到一包数据后，IMU 和点云为什么走不同路径？

`LdsLidar` 把组帧回调和 IMU 回调分别交给 `PubHandler`：

@snippet{lineno} lds_lidar.cpp livox-register-data-callbacks

SDK 数据入口先判断包类型：

@snippet{lineno} comm/pub_handler.cpp livox-imu-branch

@snippet{lineno} comm/pub_handler.cpp livox-imu-direct-delivery

<b>读末尾的 `return`</b>：IMU 解出角速度、加速度和时间后直接调用 `imu_callback_`，不会继续进入下面的原始点包队列。点云包先复制到 `RawPacket`，再入队：

@snippet{lineno} comm/pub_handler.cpp livox-raw-packet-queue

处理线程取队首并移出队列，释放锁后才解码和检查组帧：

@snippet{lineno} comm/pub_handler.cpp livox-raw-packet-consume

@snippet{lineno} comm/pub_handler.cpp livox-process-then-frame

<b>状态例子</b>：先收到 IMU A，再收到点包 B。A 可以进入独立 IMU 队列和发布线程，B 还需等待 `RawDataProcess()` 与 `CheckTimer()`。因此已经收到 IMU 消息不能证明点云已完成组帧。这里 `raw_packet_queue_` 的 `push_back` 没有容量门限；下一节所述整帧队列是另一层。

<b>边界与停止点</b>：回调注册、SDK 初始化成功或控制命令应答均不等于 ROS 点云发布。确认“类型判断→IMU return / 点包入队→取队首→组帧检查”即可停止；完整调用在 @ref livox_ros::PubHandler "PubHandler"、@ref livox_ros::LidarCommonCallback "LidarCommonCallback"、@ref livox_ros::Lds::StoragePointData() "StoragePointData()" 和 @ref livox_ros::Lds::StorageImuData() "StorageImuData()"。

@anchor livox_frame_time_reading
# 2. 配置 10 Hz 就一定每 100 ms 出帧吗？时间回退会自动修复吗？

收到每包时，先根据 `time_type` 更新同步模式。它是 `PubHandler` 的共享静态标志，不是每个 handle 独立保存的质量评估：

@snippet{lineno} comm/pub_handler.cpp livox-sync-mode-from-packet

同步分支按设备点时间检查出帧边界：

@snippet{lineno} comm/pub_handler.cpp livox-frame-sync-gates

<b>依次核对两个条件</b>：最近点时间先取整到毫秒，需满足 `recent_time_ms % publish_interval_ms_ == 0`，还要从缓存首点到最近点的差值达到容差。10 Hz 时配置间隔为 100 ms，容差是 99 ms；这不是一个每 100 ms 强制定时发送的 ROS timer。

<b>手算例子</b>：假设首点为 1000 ms，最近点为 1099 ms，虽然跨度 99 ms，模运算仍拒绝；最近点落在 1100 ms 且缓存非空时才通过这两个门。此后还需取出点、进入整帧队列并真正发布。

无同步分支改用主机时钟判断间隔：

@snippet{lineno} comm/pub_handler.cpp livox-frame-host-clock

包时间本身的选择见这里：

@snippet{lineno} comm/pub_handler.cpp livox-packet-time-source

PTP/GPS 类型直接使用包内时间；其他类型取 `high_resolution_clock` 的计数。不能将此 fallback 解释为已验证的 UTC 或保证单调的 `steady_clock`。PointCloud2 的 Header 使用缓存帧首点 `base_time`，不是发布时刻：

@snippet{lineno} lddc.cpp livox-message-header-time

点字段名虽然来自内部 `offset_time`，PointCloud2 写入时没有减去 `base_time`：

@snippet{lineno} lddc.cpp livox-pointcloud-point-time

以高精度直角坐标点解码为例，内部时间由包时间加点内间隔构造：

@snippet{lineno} comm/pub_handler.cpp livox-point-time-construction

因此 PointCloud2 保存点的完整时间计数；CustomMsg 则另做 `offset_time - base_time`。不能把两种消息的点时间字段按同一相对时间解释。

<b>回退边界</b>：同步分支的 `diff` 是无符号减法，没有在这里先拒绝“最近时间小于首点时间”。例如 1100 ms 的旧缓存后接 1000 ms 的回退点，会发生无符号回绕，不能把它理解成正确的负时间差或已清空缓存。此处也没有自动重新校时或重启设备。不要仅凭 `time_type` 就断言时间可信；启动授时门槛见 @ref livox_time_sync ，独立同步证据见 @ref livox_diagnostic_freshness_reading 。

<b>停止点</b>：能指出时间来源、模运算门限、Header 与点字段的赋值即可；完整实现见 @ref livox_ros::PubHandler::CheckTimer() "CheckTimer()"、@ref livox_ros::LidarPubHandler "点解码器"、@ref livox_ros::Lddc::InitPointcloud2Msg() "InitPointcloud2Msg()"。标准 SANY 每进程单雷达；多设备同进程时还须考虑共享同步标志的交替更新。

@anchor livox_queue_publish_reading
# 3. 已完成组帧，为什么还可能没有 ROS 点云？

组帧回调先记录诊断点云，再经 `StoragePointData()` 进入每设备的整帧队列。`Lds::PushLidarData()` 的容量分支是：

@snippet{lineno} lds.cpp livox-frame-queue-capacity

<b>对照两个分支</b>：未满才调用 `QueuePushAny`；已满时只尝试唤醒消费者，没有覆盖旧帧或入队当前帧。假设容量 4，已有 A/B/C/D，E 到达时仍满，则 E 不进入队列；不能按定位模块的“保留最新”策略理解这个驱动队列。

点云发布线程还需通过设备 `Sampling` 状态、输出格式等选择；PointCloud2 分支取出非空帧后才构造消息：

@snippet{lineno} lddc.cpp livox-frame-to-message

最终 ROS 输出及工作回调在这里发生：

@snippet{lineno} lddc.cpp livox-pointcloud-publish-progress

<b>边界与停止点</b>：`PublishPointcloud2()` 会连续取队列中现有的帧，配置 10 Hz 不意味着积压时仍严格按墙钟 100 ms 一帧发出。`publish()` 返回只是本进程调用完成，不能证明 DDS 订阅端收到。按需创建 Publisher 的日志只说明走到该路径；还要核对实际消息及其时间戳。读到 `QueuePushAny`、`QueuePop`、`publish` 三个位置即可停止；完整队列源码为 @ref ldq.cpp "ldq.cpp"，格式分发与 Publisher 创建见 @ref lddc.cpp "lddc.cpp"。观测方法见 @ref livox_troubleshooting 。

@anchor livox_shutdown_reading
# 4. 发出退出信号后，发布线程一定能结束吗？

析构函数的顺序是停止诊断定时器和心跳，再请求数据源退出、设置 future 信号、等待两条发布线程：

@snippet{lineno} driver_node.cpp livox-node-exit-order

但发布线程只有从分发函数返回后才检查 future：

@snippet{lineno} livox_ros_driver2.cpp livox-publish-thread-stop

点云分发在进入阻塞前检查退出标志；信号量的等待谓词只看 `count_`：

@snippet{lineno} lddc.cpp livox-distribute-blocking-wait

@snippet{lineno} comm/semaphore.cpp livox-semaphore-predicate

`RequestExit()` 本身仅置位：

@snippet{lineno} lds.cpp livox-request-exit-flag

<b>状态推演</b>：如果线程已在 `Wait()` 内且计数为 0，此时设置 `request_exit_` 和 future 不会直接满足 `count_ > 0`。只有另有信号使分发返回，线程才有机会检查 future。因此上面的 `join()` 顺序不能证明无数据时必定及时退出；IMU 发布线程有对应的独立等待。

<b>边界与停止点</b>：这里验证的是请求与等待之间的关系，没有实现退出修复，也没有把历史 `-11` 崩溃归因于这一处等待。2026-10-08 的退出异常与 SIGKILL 记录见 @ref livox_incident_20261008 。读到等待谓词与退出信号互不关联即可停止；后续所有权及 SDK 清理由 @ref livox_ros::Lddc::PrepareExit() "Lddc::PrepareExit()"、@ref livox_ros::LdsLidar::PrepareExit() "LdsLidar::PrepareExit()" 和 @ref driver_node.cpp "DriverNode 析构" 继续核对。
