@page livox_restart_test_20261027 双 Orin 四雷达驱动重启测试

# 测试范围与结论

2026-10-27 北京时间，用户要求根据 Obsidian 部署文档对主、从 Orin 的所有雷达持续重启测试。约定先做 10 轮驱动重启，出现异常停止并保留现场。用户确认车辆停稳、自动驾驶停用，允许中断四雷达数据及暂停定位、录包。

**第 1 轮即失败，后续 9 轮未执行。** 前、左、右、后四个驱动在退出清理时均出现 `exit code -11`（SIGSEGV）。随后按原部署入口恢复驱动，四雷达均正常出数。未执行设备软重启、域控重启、升级代码或修改设备设置。

这证明现场仍存在驱动退出崩溃问题，不是“10 轮通过”。本次没有复现此前右雷达扫描模块异常，但短时正常采样不能排除其再次发生。两个问题分别记录，见 @ref livox_incident_20261027 。

# 依据、版本和拓扑

依据知识库 `50_部署与运维/lightning-lm定位与地图数据部署.md` 的两步入口及旧 slave 保留说明，并参考 `双 Orin 与四雷达运行手册.md` 的前左右/后雷达分工。旧手册中的 `livox_ros_driver2` 启动包名需结合现有分仓部署使用 `livox_ros_driver2_core`，没有回退到旧包名。

| 项目 | 主 Orin ubuntuM | 从 Orin ubuntuS |
|---|---|---|
| 工作区 | `/home/nvidia/Sany/lightning_lm_ws` | 同左 |
| 实际入口 | `/home/nvidia/Sany/run_lidar.sh` → `sany_lidar.launch.py` | `ros2 launch livox_ros_driver2_core slave_lidar.launch.py` |
| 雷达 | 前 3.184、左 1.108、右 2.133 | 后 4.143，主机接收地址 192.168.4.101 |
| 驱动 HEAD | `c4d405e14a15f3e2018f311c1c0a3555d7dd393a` | `296e7f0951d06835779a4050e884ad6ff0180cc1` |
| common_msgs HEAD | `a6e3045097e5433f35370694d858609ef6a17a11` | `79c2a6843a4bae26d9bf7b2bb9f1b68fa67bb7b1` |
| Lightning-LM HEAD | `24b98017515053ca0b3b577897b3ee6b5092db73` | `49daa6d80551085030332618bdccde010b2f7fb0` |

两端 SDK HEAD 均为 `92a02baa44d87aa53562af4db60cfed684835d89`，域为 42、ROS_LOCALHOST_ONLY=0，Fast DDS 配置路径均为 `/home/nvidia/Documents/ros2/fastdds.xml`。两端版本不一致，因此后雷达不能作为新版 SN 参数功能验证；后雷达 SN 通过官方只读设备查询获取。

# 操作时间线与退出证据

1. 11:45:58 向 `collection_localization` 发送终端 Ctrl-C，随后确认 `run_loc_online` 已退出。录包会话此前已经消失，检查未发现 `ros2 bag record`；未强杀录包。相机、CAN、上下层控制进程保持原状，用户已确认自动驾驶停用。
2. 11:46:36 向两端原 launch 父进程发送 SIGINT：主 PID 29894，从 PID 48018。等待 20 秒，保存完整日志。
3. Launch 在约 5 秒后升级发送 SIGTERM，四个节点收到终止信号，打印 SDK 清理结束，随后退出码均为 -11。
4. 确认旧驱动进程消失后，恢复原入口；主机保留 tmux 名 `collection_lidar`，从机为 `collection_rear_lidar`。启动输出改存本次测试目录，详见下文。

| 角色 | 原 PID | 清理日志 | 退出结果 |
|---|---|---|---|
| 前 | 29924 | `Livox Lidar SDK Deinit completely!`、`lddc destory` | -11 |
| 左 | 29926 | 同上 | -11 |
| 右 | 29928 | 同上 | -11 |
| 后 | 48037 | 同上，另有 `lds destory` | -11 |

**信号方法边界**：本轮只向 launch 父进程发送 SIGINT，不等同于终端向整个前台进程组发送 Ctrl-C。因此不能把“5 秒后升级 SIGTERM”单独认定为节点处理 SIGINT 卡死；已确认的失败是节点收到 SIGTERM 后在清理阶段 SIGSEGV。下一步应分别验证终端 Ctrl-C、直接 SIGTERM 和退出对象生命周期。

从机旧版同样崩溃，说明该现象并非新版 SN 参数功能独有；仍需线程栈或 core 证明具体崩溃位置，不能直接写成已确认 double-free 或悬空指针。现场未发现本轮可用的 core，未修改系统 core 配置，也未修复代码。

# 重新启动后的实测

各 Orin 本地订阅其连接雷达的 PointCloud2 和 IMU，测量窗口 20 秒，按实际首末接收时刻计算频率。这里只证明本地接收，不代表跨 Orin DDS 链路通过。

| 雷达 | 点云数 / Hz | IMU 数 / Hz | 末帧点数 | Header 不递增次数 |
|---|---|---|---|---|
| 前 | 183 / 10.002 | 3667 / 200.005 | 19968 | 0 |
| 左 | 183 / 10.002 | 3666 / 199.995 | 20064 | 0 |
| 右 | 183 / 9.998 | 3667 / 200.014 | 19968 | 0 |
| 后 | 200 / 9.998 | 4001 / 200.001 | 19968 | 0 |

四台 `cur_work_state=1`，HMS 均有 `0x04050002`（曾发生时间同步异常），没有读到右雷达此前的 `0x02160003`、`0x02100003`。不能写成“所有告警清空”。后雷达一次 `lidar_diag_status` 查询超时，该字段本轮记为未知；工作状态、HMS 和实际接收另有成功证据。

四台 SN：前 `ARMCP580033584`、左 `ARMCP570032908`、右 `ARMCP580033938`、后 `ARMCP580033833`。末次点云 Header 到接收约 0.108–0.113 秒，IMU 约 0.0009–0.0013 秒，均为本次抽样。

# 证据与现场交接

两端分别保存同名目录 `/home/nvidia/Sany/lidar_restart_test_20261027/`：

- `baseline_launch.log`：停止前日志副本。
- `round01_stop.log`：含本轮退出故障的完整原启动日志。
- `round01_start.log`：重新启动的输出；恢复后的驱动持续写入此文件。
- `round01_receive.json`：本地逐话题接收计数、频率和时间戳统计。
- `round01_device_state.jsonl`：官方只读参数查询返回码及原始数据。
- `restart_entry.sh`：本次恢复入口，分别调用原主机根脚本或从机 launch。

本地证据归档于工作区 `_work/lidar_restart_test_20261027/master/` 和 `slave/`，不包含凭据或完整环境变量。原主机日志 `/home/nvidia/project/yk_ws/collection_toolkit29/logs/lidar_20261027_100815.log`、原从机日志 `/home/nvidia/.local/state/vla_collection/rear_lidar.log` 保留。

结束状态：四雷达驱动已恢复运行；定位保持停止，录包未启动，未自动恢复驾驶任务。当前不能宣称重启可靠性验收通过。后续先定位并修复退出崩溃，再按相同版本基线重新开展完整 10 轮测试，并分别报告终端正常退出、采样恢复和设备错误状态。
