@page livox_heartbeat 功能安全心跳与健康边界

# 配置与接口

心跳类型为 `diagnostic_monitor_interfaces/msg/NodeHeartbeat`，由 @ref livox_ros::functional_safety::HeartbeatPublisher 的独立线程定期发布。

| 雷达 | node_id | Topic |
|---|---:|---|
| 前 | 7 | `/diagnostics/heartbeat/livox_front` |
| 左 | 8 | `/diagnostics/heartbeat/livox_left` |
| 右 | 9 | `/diagnostics/heartbeat/livox_right` |
| 后（独立拓扑） | 12 | `/diagnostics/heartbeat/livox_rear` |

Launch 当前设置 `heartbeat_period_ms=100`、depth=1、best_effort、volatile。`node_id=0` 时节点不启用心跳；启用时 Topic 必须为绝对路径，周期与 depth 必须为正，reliability 为 `best_effort` 或 `reliable`。

每台驱动在自己的 `active_fault` 中上报自身九类当前 DTC，检测周期 100 ms。
FAIL 加入，只有明确且新鲜的 PASS 才清除，UNKNOWN/过期保留；历史通信/同步事件
（各角色 base+12/base+13）仅保留在 `diagnostic_state` 的十一项原始观测中。
活动集合不跨进程重启持久化，诊断中心须同时处理 boot_id 和未知状态。
独立聚合心跳 `/diagnostics/heartbeat/livox_dtc` 与未登记的 ID 13 已移除。

# 序列与状态

- `boot_id` 是启动时生成的非零随机值，可区分进程重启。
- `heartbeat_seq` 随心跳发布递增。
- `work_seq` 仅随实际点云发布回调递增，`DriverNode` 的工作回调会将状态改为 Running；诊断检测和心跳发送不推进它。
- 正常心跳只能证明心跳线程还在运行，仍需结合设备诊断和点云时间戳判断传感器健康。

代码依据：`src/livox_ros_driver2.cpp` 的点云发布回调和诊断定时器、`src/lidar_dtc_reporting.h`，及 `src/functional_safety_heartbeat.cpp` 的 `RecordWork`、`PublishNow`、`Run`。

# 现场判定

```bash
ros2 topic echo /diagnostics/heartbeat/livox_front \
  --once --qos-reliability best_effort
```

结合实际点云计数/频率、IMU 和 Header 时间判定传感器健康。Topic、心跳、IMU 正常而点云缺失的典型案例见 @ref livox_incident_20261008 。
