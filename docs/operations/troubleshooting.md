@page livox_troubleshooting 故障定位与授时检查

# 按数据链路分层定位

| 证据 | 下一步 |
|---|---|
| 设备不能 ping，控制初始化失败 | 网卡、物理链路、主机/设备 IP、路由和重复进程 |
| IMU/心跳可见，点云 Publisher 缺失 | 优先检查点云 UDP、雷达工作状态、HMS 与时间同步 |
| 已有点云 UDP，驱动未进入点云发布 | 安装配置、实际 SDK 库、Socket、队列与 PubHandler 组帧 |
| 驱动发布日志存在，但同环境订阅不到 | ROS_DOMAIN_ID、DDS 实现、接口白名单、QoS、跨机发现 |
| Topic 可见但收到 0 条消息 | Publisher 数量和实际订阅；订阅者也会创建可见 Topic |
| 退出码 -9，之前有 SIGINT/SIGTERM 超时 | 这是 Launch 升级终止的结果，保留退出等待日志 |
| 正常采集后退出码 -11 | 保存退出清理日志；与启动无点云分别定位 |

# 环境、安装树与网络

检查终端和驱动使用同一环境：

```bash
source "$HOME/.config/sany/workspace.bash"
set +u
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ros2 pkg prefix livox_ros_driver2_core
pgrep -af 'livox_ros_driver2_node|ros2 launch livox_ros_driver2_core'
ip -br addr
ip route
ss -ulnp
ros2 topic list --no-daemon --spin-time 3
```

需要时从 `/proc/<pid>/environ` 定向读取 ROS_DOMAIN_ID、DDS 配置路径、AMENT_PREFIX_PATH；不要整份导出进程环境以免包含凭据。`/proc/<pid>/maps` 可确认 SDK 实际加载路径。

# 区分点云和 IMU UDP

保持驱动运行并抓取：

```bash
sudo tcpdump -ni any -nn \
  'udp and ((src port 56300) or (src port 56400))'
```

`ping` 成功只说明基本 IP 连通。点云和 IMU 独立输出：前三路目标端口分别为 56301、56401。没有抓包权限时，可比较 `/sys/class/net/<iface>/statistics/rx_bytes` 增量，但总流量只能辅助判断，不能代替包类型证据。

2026-10-08 故障时三路约 21 KB/s，恢复后约 3 MB/s；这些数值是该现场观测，不作为通用阈值。

# 授时与告警

```bash
date -Ins
timedatectl
chronyc tracking
chronyc sources -v
pgrep -af 'ptp4l|phc2sys|chronyd'
journalctl -b --no-pager -o short-monotonic \
  -u chrony -u ptp_master_eth1 -u ptp_master_eth1_sys
```

按实际服务名称读取其他网卡日志。重点看启动次序、`System clock was stepped`、PHC 回退与 PTP 选主；当前已经同步不等于启动时没有回退。

设备状态可通过 Livox 官方只读参数查询读取 `cur_work_state`、点云目标地址、`time_sync_type`、`last_sync_time`、`time_offset` 和 `hms_code`。`0x04050002` 为时间同步异常 Warning；HMS 可能保留历史事件，必须联合日志和恢复试验判断。

[Livox 官方协议](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/livox_eth_protocol_mid360.md) 说明默认时间过滤模式下时间回退可能造成点云中断。[HMS 表](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/hms_code_mid360.rst) 给出告警含义。固件不支持的查询会返回 `0x20`，不能把未返回的字段当成 0 或关闭。

# 恢复与防复发

确认车辆停稳、允许中断数据、当前授时稳定后，可先软重启一台雷达验证，再处理其他设备；避免直接重启域控或反复拉起驱动替代设备状态检查。设备软重启不等于恢复出厂设置，现场操作使用官方 SDK 或 Viewer。

防复发应检查 RTC 的 UTC/本地时间模式，完成开机初始校时后才让 PTP 对外授时，并限制运行期时钟大幅跳变。配置 `After=chrony.service` 本身不证明初始校时完成，需要等待实际同步条件。

当前时间已正确时，可由现场管理员将 RTC 改为 UTC：

```bash
sudo timedatectl set-local-rtc 0
```

此命令默认把系统时间写入 RTC；不加 `--adjust-system-clock`。参见 [systemd 文档](https://github.com/systemd/systemd/blob/v249/man/timedatectl.xml)。Chrony 的负数 `makestep` 更新次数限制允许长期 step，调整策略需与启动等待和失联保持一起评审，见 [Chrony 4.2 文档](https://chrony-project.org/doc/4.2/chrony.conf.html#makestep)。本手册记录建议，尚未修改现场系统配置。
