@page livox_incident_20261027 sany-03-master 右雷达扫描模块异常与恢复

# 结论与范围

2026-10-27 北京时间，用户更新代码并通过 tmux 启动后，定位输入检查无法通过。现场实际为前、左雷达正常，右雷达 `192.168.2.133` 自报 `cur_work_state=4`（`kLivoxLidarError`），HMS 包含扫描模块异常 `0x02160003`、`0x02100003`，没有点云和 IMU 发布。设备发现、SN 查询和设置工作模式的应答均成功，不能据此判断正在正常采样。

用户确认车辆停稳并授权仅软重启右雷达后，于 **10:03:39** 发送一次设备软重启请求，应答 `ret_code=0`。原三路驱动进程保持运行；右雷达恢复采样，HMS 清空，六路实际订阅和定位输入检查通过。未修改代码、配置、DDS、授时服务，未重启域控或前、左雷达。

直接故障定位为**右雷达内部扫描模块异常状态导致停流**。扫描模块异常的更深触发原因尚未确定；一次软重启恢复不能证明硬件永久正常，也不能证明此次代码更新触发了故障。当前证据不支持把 SN 参数改动或 tmux 作为原因。

# 环境与版本

- 主机：`sany-03-master` / `ubuntuM`，ROS Humble，域 `42`，Fast DDS。
- 工作区：`/home/nvidia/Sany/lightning_lm_ws`。
- Lightning-LM：`24b98017515053ca0b3b577897b3ee6b5092db73`。
- 驱动：`c4d405e14a15f3e2018f311c1c0a3555d7dd393a`。
- common_msgs：`a6e3045097e5433f35370694d858609ef6a17a11`。
- 上述三个现场源码仓库检查时均无未提交修改；版本哈希与运行数据分别检查，不以源码版本代替运行验收。
- 启动 ID：`7496fedf-39c9-4ac4-a543-bb681fed4b20`。
- VIN：`SANLV77EXSH000858`。

| 角色 | IP | 实测 SN | 恢复前状态 |
|---|---|---|---|
| 前 | 192.168.3.184 | ARMCP580033584 | 正常采样 |
| 左 | 192.168.1.108 | ARMCP570032908 | 正常采样 |
| 右 | 192.168.2.133 | ARMCP580033938 | Error，无点云/IMU |

# 原始证据与排除过程

当前 tmux 会话是 `collection_lidar`，标准输出已重定向，空白 pane 不是无进程的证据。通过 `/proc/25023/fd/1` 找到真实日志：

```text
/home/nvidia/project/yk_ws/collection_toolkit29/logs/lidar_20261027_095730.log
/home/nvidia/project/yk_ws/collection_toolkit29/logs/localization_20261027_095736.log
/home/nvidia/Sany/runs/sany_lidar_20261027_095736/logs/input_check.log
```

驱动 PID 为前 `25055`、左 `25057`、右 `25059`。三路均有 SN 确认、设置点云格式成功、切换 Normal 成功、启用 IMU 成功日志。只有前、左出现点云和 IMU 发布日志。只读订阅 12 秒得到前点云 115 帧、左 115 帧、主 IMU 1112 条、右点云 **0 帧**，输入检查超时退出。原定位启动已经在输入门槛处退出，恢复雷达不会自动重启该定位任务。

右雷达 ping 2/2 成功，设备只读查询返回的目标是 `192.168.2.100:56301`（点云）与 `192.168.2.100:56401`（IMU），与 eth2 主机地址和驱动 UDP 监听一致。前雷达同样查询为正常状态，形成对照。

查询使用官方参数命令 `0x0101`，验证源 IP、命令、序号、长度及 CRC，只读、不写设备设置：

| 参数键 | 右雷达恢复前 | 解释 |
|---|---|---|
| `0x001A` work_mode | 1 | 请求工作模式为 Normal，不等于实际状态正常 |
| `0x001C` imu_data_en | 1 | IMU 配置启用，不等于正在出数 |
| `0x8006` cur_work_state | 4 | 实际 Error；前雷达为 1 |
| `0x800E` lidar_diag_status | 0 | 此字段不能覆盖工作状态和 HMS 的异常证据 |
| `0x8011` hms_code | `0x02160003`, `0x02100003`, `0x04050002` | 扫描模块 Error，以及时间同步曾异常的 Warning |
| `0x0003`, `0x800D` | 返回码 `0x20` | 本机固件不支持查询，不能解释为值 0 |

HMS 数据段原始 32 字节为：

```text
03001602 03001002 02000504 00000000 00000000 00000000 00000000 00000000
```

按小端 uint32 解码。[Livox 官方 HMS 表](https://github.com/Livox-SDK/livox_wiki_en/blob/master/source/tutorials/new_product/mid360/hms_code_mid360.rst) 将 `0x0210–0x0219` 的 Error 解释为扫描模块异常、系统尝试恢复；长期持续时建议重启设备。`0x0405` 仅表示时间同步曾异常，不能单独确定本次扫描模块异常的触发原因。

无 sudo 权限，未获得 tcpdump 抓包或 gdb 线程栈；因此不把网卡计数或线程等待位置写成已证实的丢包/死锁原因。

# 恢复措施与复验

1. 保留故障日志、设备状态、HMS 和订阅计数。确认车辆停稳且允许中断右雷达数据。
2. 检查当前 Chrony/PTP 状态。操作前 Chrony `Leap status=Normal`，系统相对 NTP 约慢 12 微秒，eth2 PHC 对系统偏差抽样在几十纳秒范围内；这只是当时状态，不代表整次运行没有授时异常。
3. 仅向 `192.168.2.133:56100` 发送一次 `0x0200` 软重启，延迟字段 100 ms，与 SDK `CommandImpl::LivoxLidarRequestReboot()` 一致。应答 `ret_code=0`，Unix 时间纳秒 `1793066619611169882`。无恢复出厂、IP 或其他持久参数写入。回执丢失时应先查状态和数据，不盲目重发。
4. 保持原驱动运行，等待设备重新上线。复查 `cur_work_state=1`，右 HMS 八个 uint32 全为 0；前、左仍正常且各保留 `0x04050002`，不能写成三台全部告警清零。
5. 六路同时实际订阅 20 秒，发现后的有效接收窗口约 18 秒，结果如下。

| 角色 | 点云帧数 / Hz | IMU 条数 / Hz | 末帧点数 | 时间戳不递增次数 |
|---|---|---|---|---|
| 前 | 180 / 10.002 | 3596 / 200.069 | 19968 | 0 |
| 左 | 180 / 10.002 | 3599 / 200.007 | 19968 | 0 |
| 右 | 180 / 10.005 | 3599 / 200.009 | 19968 | 0 |

末次点云 Header 到接收约 0.105–0.109 秒，IMU 约 0.0007–0.0015 秒；这是本次抽样值。随后运行同版本输入检查器：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/Sany/lightning_lm_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/nvidia/Documents/ros2/fastdds.xml
ros2 run lightning_lm check_localization_inputs --timeout 10 \
  --lidar /livox/lidar_192_168_3_184 \
  --lidar /livox/lidar_192_168_1_108 \
  --lidar /livox/lidar_192_168_2_133 \
  --imu /livox/imu_192_168_3_184 \
  --optional-wheel-speed /SpeThrCAN4_topic
ros2 run livox_ros_driver2_core collect_lidar_identity.py \
  --roles front left right --timeout 5
```

输入检查输出 `Sensor delivery check passed for 4 required topics`；CAN 布局识别成功、`malformed_CAN=0`、抽样 CAN-IMU 差约 -14.4 ms。SN 参数仍正常。以上是雷达恢复与定位输入验收，**不等于定位算法、地图匹配或行驶验收**。原定位入口已超时退出，需要现场重新执行 `/home/nvidia/Sany/run_loc.sh`；本次未自动启动定位任务。

# 独立风险与后续处理

- 本次启动 Chrony 在单调时间 184.078 秒跳变约 `1793063086.795` 秒；PTP guard 在 203.156 秒才报告有效时间、禁止后续自动跳时并初始化 PHC，PTP 在 203.169 秒启动、210.982 秒进入 MASTER。已看到门槛生效，不能照搬 2026-10-14“先授时后跳时”的结论。
- eth2 PTP 在单调时间约 1338、1629、1706 秒出现 `timed out while polling for tx timestamp`、`send sync failed`、`MASTER to FAULTY`，随后恢复 MASTER。此为独立的授时可靠性问题；没有证据链证明它导致扫描模块 Error。本次未调整 `tx_timestamp_timeout`、系统时钟或网卡驱动，后续需结合完整 PTP 日志与传感器状态时间线专项复查。
- 09:42 旧启动日志中，前/左正常采集后退出有 `-11`，右曾在 SIGTERM 后等待超时被 `-9` 终止。这些是退出清理问题，应独立分析；本次未重新制造退出或声称已经修复。
- 若右雷达再次出现 `0x0210–0x0219` 且停流，保存 SN、固件版本、HMS、设备状态、供电及环境情况，联系厂商检查扫描模块。不要用周期性重启掩盖复发，不把查询到 SN 当作设备可用信号。

关联排查入口：@ref livox_troubleshooting ，授时门槛：@ref livox_time_sync 。
