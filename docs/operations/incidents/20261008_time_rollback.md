@page livox_incident_20261008 sany-03-master 开机时间回退与点云恢复

# 现象与版本

2026-10-08（北京时间），`sany-03-master` 上前、左、右三台设备能 ping，驱动发现与控制成功，ROS 中有 IMU 和心跳，但没有点云 Publisher/实际点云。该次加载的工作区与源码版本：

| 项目 | 现场值 |
|---|---|
| 工作区 | `/home/nvidia/Sany/lightning_lm_ws` |
| 驱动 | `fbbef2c45a8817279421d7edddae18c416404288` |
| SDK2 | `92a02baa44d87aa53562af4db60cfed684835d89` |
| 定位 | `5597e254a5d123ce95d2ba29d7ecd92903358f6e` |
| ROS | Humble，ROS_DOMAIN_ID=42 |
| DDS profile | `/home/nvidia/Documents/ros2/fastdds.xml` |

驱动加载统一安装树中的 SDK shared library，三路点云目标 IP/端口与主机网卡一致。未修改驱动、SDK、JSON、DDS 或定位源码。

# 关键证据

1. 三路 NIC 在故障时各约 21 KB/s；当前 IMU 可实际收到，驱动日志只有 IMU publish 记录。
2. 三台设备只读查询均为 Sampling、PTP 同步、正确点云目标；HMS 都含 `0x04050002`，左/右另含 GPS 异常历史告警。
3. Chrony 开机日志明确记录时间回拨：

```text
[25.476] ptp4l: assuming the grand master role
[35.508] chronyd: System clock wrong by -28800.660176 seconds
[35.508] chronyd: System clock was stepped by -28800.660176 seconds
[36.095] phc2sys: eth1 sys offset 28800660175845 s1
```

4. RTC 被配置为本地时间；三个 PTP/PHC 服务仅 `After=network.target`，在 Chrony 完成初始校时前已运行。
5. 当前 Chrony 与 PTP 后来已稳定，但点云未自动恢复。官方协议支持时间回退造成点云中断的解释；告警含义见 @ref livox_troubleshooting 。

# 恢复试验

用户确认车辆停稳并分别授权后，先软重启前雷达，仅该路流量从约 21 KB/s 恢复到约 3 MB/s；临时启动前驱动收到 131 帧 PointCloud2。随后软重启左/右，最终使用部署的 `run_lidar.sh` 临时启动三路进行测量。

丢弃启动阶段后，连续测量 12 s：

| 雷达 | 点云数量 | 点云频率 | 末帧点数 | IMU 数量 / 频率 |
|---|---:|---:|---:|---|
| 前 184 | 120 | 10.002 Hz | 19968 | 2400 / 200.009 Hz |
| 左 108 | 120 | 9.999 Hz | 19968 | 2400 / 199.999 Hz |
| 右 133 | 120 | 10.000 Hz | 19968 | 2401 / 199.998 Hz |

三台设备最终 HMS 均为空。临时驱动在验证后结束。左雷达的重启回执曾超时，但后续流量与只读状态确认已经恢复，因此没有重复发送重启命令。

证据保存在该主机本次启动的 system journal 与 `/home/nvidia/.ros/log/`：三路最终验证 Launch 进程为 70236，驱动进程为 70284/70286/70288，可按这些 PID 与本次启动时间定位日志。日志存在性受现场清理策略影响。

# 结论与未完成项

本次点云中断定位到开机时钟回退经 PTP 传给设备；在时钟稳定后软重启设备，三路点云均恢复。RTC 本地时间配置与缺少首次校时等待是防复发需要处理的配置问题；8 h 偏差的完整开机 RTC 加载机制尚未单独实验验证。

系统配置没有改动：RTC UTC 修正、PTP 启动门槛、Chrony 运行期 step 与失联保持仍需实施并做冷启动验证。禁止将该次恢复试验表述为这些配置已修复，或推断所有“仅有 IMU”故障均由时间回退造成。

此外，临时驱动正常采集后退出时三节点均报 `-11`；此前无点云时存在退出超时后 SIGKILL。这些退出问题尚未修复，不影响本次恢复后已测量的采集结果。
