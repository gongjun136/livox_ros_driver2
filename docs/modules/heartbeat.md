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

# 序列与状态

- `boot_id` 是启动时生成的非零随机值，可区分进程重启。
- `heartbeat_seq` 随心跳发布递增。
- `work_seq` 随数据工作回调递增，`DriverNode` 的工作回调会将状态改为 Running。
- 正常心跳只能证明心跳线程还在运行。工作计数也不是点云专用计数；IMU 有数据时仍可能递增，不能据此认定点云正常。

代码依据：`src/livox_ros_driver2.cpp` 中传给数据源的工作回调，及 `src/functional_safety_heartbeat.cpp` 的 `RecordWork`、`PublishNow`、`Run`。

# 现场判定

```bash
ros2 topic echo /diagnostics/heartbeat/livox_front \
  --once --qos-reliability best_effort
```

结合实际点云计数/频率、IMU 和 Header 时间判定传感器健康。Topic、心跳、IMU 正常而点云缺失的典型案例见 @ref livox_incident_20261008 。
