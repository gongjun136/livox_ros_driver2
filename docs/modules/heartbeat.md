@page livox_heartbeat 功能安全心跳与健康边界

心跳、原始设备诊断和实际点云分别提供不同证据。本页先核对三条验证路径，再查配置与现场命令；例子为状态推演，不替代实机验收。

@anchor livox_heartbeat_work_reading
# 1. 心跳持续更新，就代表点云还在发布吗？

发送心跳时，一个计数器推进，另一个只读取：

@snippet{lineno} functional_safety_heartbeat.cpp livox-heartbeat-publication

<b>比较 `Advance(heartbeat_seq_)` 与 `work_seq_.load()`</b>：心跳独立线程的发送不会自动增加工作序列。PointCloud2 的发布调用后才触发工作回调：

@snippet{lineno} lddc.cpp livox-pointcloud-publish-progress

@snippet{lineno} livox_ros_driver2.cpp livox-heartbeat-work-callback

`RecordWork()` 增加工作序列，随后设置 Running。诊断定时器和 IMU 发布均没有走这条点云工作回调。

<b>状态例子</b>：同一 `boot_id` 下，连续收到 `(heartbeat_seq,work_seq)=(50,12),(51,12),(52,12)`，只能确认心跳继续产生，不能证明这段时间有新的点云发布。若下一次为 `(53,13)`，则本进程完成过一次点云发布调用；仍不等于定位订阅端已收到或点云时间可信。

<b>边界与停止点</b>：Running 在点云回调中置位，不是每次心跳重新计算的设备健康结论；点云停止后也可能保留该值。新 `boot_id` 要作为新进程处理，不能把重启后的序列直接与旧值比较。读完计数器和工作回调即可停止；完整源码见 @ref livox_ros::functional_safety::HeartbeatPublisher "HeartbeatPublisher"、@ref livox_ros::DriverNode "DriverNode"、@ref livox_ros::Lddc::PublishPointcloud2Data() "PublishPointcloud2Data()"。

@anchor livox_diagnostic_freshness_reading
# 2. 有 UDP 包、IMU 或重复点云，为什么设备仍可能不可用？

诊断分别记录点包到达、有效点云及设备状态。`ObserveCloud()` 只有同时满足以下条件才刷新有效点云时刻：

@snippet{lineno} lidar_dtc_monitor.h livox-diagnostic-cloud-progress

<b>看严格大于号和点数条件</b>：`stamp` 必须大于上次有效时间，且有效点数达到门限。仅重复发送相同 Header 时间不能刷新。调用者只把有限 XYZ 且距离平方大于 `0.0001` 的点计为有效点，设备 handle 也必须匹配该角色配置。

最终可用性同时检查四种证据：

@snippet{lineno} lidar_dtc_monitor.h livox-diagnostic-usability

`valid_fresh`、`packet_fresh` 是点云和点包；`hms_fresh`、`state_fresh` 是设备诊断及工作状态。只有这些都新鲜、`work_state_==1` 且未先命中 fatal 分支，才进入 `kUsable`。IMU 包不会刷新诊断点包时刻。

<b>状态例子</b>：假设配置 `data_timeout=500 ms`，已见过有效点云。此后点包仍不断到达，但点云时间一直等于旧值，超过 500 ms 后 `valid_fresh=false`，设备成为 `kUnusable`。恢复需要时间继续前进且有效点数足够，并同时满足状态/HMS 等条件；不能靠心跳恢复或新到一个 IMU 包清除。

同步质量也使用独立证据，不能只看数据包的时间类型：

@snippet{lineno} lidar_dtc_monitor.h livox-ptp-evidence-advance

`last_sync` 回退会取消验证；重复相同 `last_sync` 不会刷新 `sync_time_`。例如状态每秒查询一次，但最后同步时刻冻结，持续查询本身不能让 PTP 证据永久有效。当前 `Evaluate()` 仅在 PTP 证据新鲜时，排除未使用 GPS/PPS 源的特定 Warning；PTP 故障、Error/Fatal 与其他原因仍分别判断。有界同步告警另有连续推进、偏差及年龄条件，不等于直接清除原始 DTC。

<b>边界与停止点</b>：首次尚无完整证据时默认 UNKNOWN；启动宽限影响何时把缺失数据判为不可用，不制造 PASS。读到“什么能刷新时间、过期时走哪个分支”即可停止；完整实现见 @ref livox_ros::dtc::Monitor "设备监测器"、@ref livox_ros::LdsLidar::ObserveDiagnosticCloud() "有效点过滤"、@ref livox_ros::LdsLidar::DiagnosticPacketCallback() "点包观察"。主机开机授时另见 @ref livox_ptp_gate_reading "PTP 启动门槛"。

@anchor livox_diagnostic_faults_reading
# 3. diagnostic_state 有 11 项，为什么心跳只带当前故障，UNKNOWN 又为何不清码？

诊断定时器将监测快照转成消息；先把所有观测加入 `diagnostic_state`，再选出属于当前故障的部分：

@snippet{lineno} lidar_dtc_reporting.h livox-diagnostic-current-vs-history

<b>读 `push_back` 与 `continue` 的顺序</b>：`kCategories=11`，最后两类历史通信/同步项仍进入原始观测；`i>=kCategories-2` 后跳过心跳故障集合。前九类中，FAIL 进入 `failed`，PASS 进入 `passed`，UNKNOWN 不进入任一集合。

心跳按下列方式保持活动集合：

@snippet{lineno} functional_safety_heartbeat.cpp livox-heartbeat-fault-latch

先删除 passed，再插入 failed，同一批若同时出现两种条件则 FAIL 优先。假设当前故障码 C 已因 FAIL 加入，下一次变成 UNKNOWN，两个集合都不包含 C，旧活动码仍保留；有明确 PASS 后才移除。若某个历史项为 FAIL，它仍可出现在 `diagnostic_state`，但不会因此进入心跳 `active_fault`。

<b>边界与停止点</b>：空活动集合不能证明检测已经完成，保留旧码也不表示现在又测得一次新 FAIL。`comm_header.seq` 表示诊断消息发送进度，`evaluation_seq=hms_seq` 与 `evaluation_age_us` 才对应 HMS 证据；不能靠反复发消息掩盖证据冻结。活动集合只在本进程中保持，外部诊断中心负责跨重启与消抖策略。读到历史项过滤和集合更新即可停止；完整源码：@ref lidar_dtc_reporting.h "诊断消息转换"、@ref livox_ros::functional_safety::HeartbeatPublisher::UpdateFaults() "UpdateFaults()"。

# 配置与接口

心跳类型为 `diagnostic_monitor_interfaces/msg/NodeHeartbeat`，由独立线程发布。

| 雷达 | node_id | Topic |
|---|---:|---|
| 前 | 7 | `/diagnostics/heartbeat/livox_front` |
| 左 | 8 | `/diagnostics/heartbeat/livox_left` |
| 右 | 9 | `/diagnostics/heartbeat/livox_right` |
| 后 | 12 | `/diagnostics/heartbeat/livox_rear` |

Launch 当前设置 `heartbeat_period_ms=100`、depth=1、best_effort、volatile。`node_id=0` 时节点不启用心跳；启用时 Topic 必须为绝对路径，周期与 depth 必须为正，reliability 为 `best_effort` 或 `reliable`。每台驱动在自己的心跳上报自身当前 DTC；独立聚合心跳 `/diagnostics/heartbeat/livox_dtc` 与未登记 ID 13 已移除。

# 现场判定

```bash
ros2 topic echo /diagnostics/heartbeat/livox_front \
  --once --qos-reliability best_effort
```

同时观察真实点云计数、Header 时间进展、IMU 与设备诊断，不能只看 Topic 或心跳频率。只有 IMU 无点云的分层排查见 @ref livox_troubleshooting ，历史证据见 @ref livox_incident_20261008 。
