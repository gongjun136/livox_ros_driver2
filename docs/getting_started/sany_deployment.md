@page livox_sany_deployment 主 Orin 雷达部署

升级后的角色驱动自动发布 SDK 设备 SN。设备信息话题、缓存语义和一键导出见
@ref livox_configuration "雷达 SN 与设备身份"。须配套更新 common_msgs 的接口后重编驱动；
实机 SN 与原数据链路应在驱动重启后的维护窗口核验。

@anchor livox_launch_layout_reading
# 1. 自动入口怎样决定三台或四台，VIN 又检查什么？

先核对 `sany_lidar.launch.py` 中的两个不同判断：

@snippet{lineno} sany_lidar.launch.py livox-launch-layout

<b>按变量读</b>：`VIN` 只检查是否属于已登记集合；具体启动哪个 Launch 由 `LIDAR` 查表决定。`MASTER AND SLAVE` 选择三台的 `master_lidar.launch.py`，`MASTER` 选择复用前三台并增加后雷达的 `master_4lidar.launch.py`。缺文件、解析失败、未知 VIN 或其他 LIDAR 值会抛出异常。

<b>输入例子</b>：有效 VIN 配合 `LIDAR: MASTER`，进入四雷达入口；只把 LIDAR 改成 `MASTER AND SLAVE`，进入三雷达入口。这不是根据发现了几台在线设备自动缩减拓扑。四雷达入口还把后雷达 JSON 覆盖为 `MID360s_config_rear_master.json`，使用主 Orin 的接收地址。

<b>边界与停止点</b>：这些选择只决定启动哪些驱动，不自动修改定位器使用哪些雷达，也不验证设备已到包。显式 `master_lidar.launch.py` / `slave_lidar.launch.py` 不经过这段车辆文件判断。读到 `layouts[vehicle['LIDAR']]` 的 Include 即可停止；完整源码：@ref sany_lidar.launch.py "自动入口"、@ref master_lidar.launch.py "前三台"、@ref master_4lidar.launch.py "四台组合"、@ref lidar_rear.launch.py "后雷达参数"。

@anchor livox_role_binding_reading
# 2. 心跳 ID 怎样绑定诊断角色，为什么要求一个进程一台雷达？

ROS 2 构造函数按固定表推导默认角色，再决定是否启用设备诊断：

@snippet{lineno} livox_ros_driver2.cpp livox-diagnostic-role

<b>注意数组顺序</b>：角色是 `front/rear/left/right = 0/1/2/3`，对应心跳 ID `7/12/8/9`。例如 ID 8 默认得到角色 2（left），不是角色 8。显式 `dtc.role` 可覆盖默认值，因此部署还必须核对 Launch 的 ID、角色、话题与设备 JSON 是否一致。

启用诊断后，读取用户 JSON 时还有这条约束：

@snippet{lineno} lds_lidar.cpp livox-role-single-device

`user_configs.size()!=1` 返回失败；通过后只把这一设备的 handle 交给诊断观察器。假设某个角色进程的 JSON 列出两台设备，它不能用同一角色的诊断快照同时表示两台。

<b>边界与停止点</b>：这里只有配置数量及角色范围校验，不能证明实际网卡或雷达 IP 正确。`InitLdsLidar()` 失败时节点记录错误并设置心跳 Fault，不能把 ROS 进程仍在或 topic 已出现当作初始化成功。读完角色表和单设备条件即可进入 @ref livox_packet_routes_reading "SDK 数据链路"；完整实现见 @ref livox_ros::DriverNode "DriverNode"、@ref livox_ros::LdsLidar::InitLivoxLidar() "InitLivoxLidar()"，参数字段见 @ref livox_configuration 。

# 网络与拓扑

旧车 master 使用前、左、右雷达；新车四台全部接入 master。自动入口读取 `/opt/conf/vehicle_config.yaml` 的 VIN 和 LIDAR，`MASTER AND SLAVE` 启动 master 三台，`MASTER` 启动四台。下表是旧车配置及 2026-10-08 `sany-03-master` 的现场核对结果；其他车辆先核对自己的网卡和 JSON。

```bash
ros2 launch livox_ros_driver2_core sany_lidar.launch.py
# WSL/测试文件路径覆盖：
ros2 launch livox_ros_driver2_core sany_lidar.launch.py vehicle_config:=/absolute/vehicle_config.yaml
```

已登记 VIN：旧车 `SANLV77EXSH000858`，新车 `SANLU77E8TH000006`。新车 VIN 于 2026-10-13 按用户确认更正，此前登记的 `SW956ELCG0858` 对应同一台车；识别以真实 VIN 为准。文件缺失、未知 VIN 或非法 LIDAR 时拒绝自动启动。原 master/slave 显式 launch 不依赖此文件，保持原启动方式。

| 雷达 | 网卡 | 主机 IP | 雷达 IP | 点云 Topic | IMU Topic |
|---|---|---|---|---|---|
| 前 | eth3 | 192.168.3.100 | 192.168.3.184 | `/livox/lidar_192_168_3_184` | `/livox/imu_192_168_3_184` |
| 左 | eth1 | 192.168.1.100 | 192.168.1.108 | `/livox/lidar_192_168_1_108` | `/livox/imu_192_168_1_108` |
| 右 | eth2 | 192.168.2.100 | 192.168.2.133 | `/livox/lidar_192_168_2_133` | `/livox/imu_192_168_2_133` |

主机运维/DDS 接口在该现场为 `eth10 / 192.168.44.100`。传感器 UDP 与 ROS DDS 是两条链路，分别检查它们的地址和端口。

# 开机授时前置条件

主 Orin 应先按 @ref livox_time_sync 安装并验收时间启动门槛，确保 PTP 不会在系统尚未校时、PHC 尚未初始化时对外授时。运行 `run_lidar.sh` 不能替代这个系统级前置条件。

# 第一步：启动雷达

保存以下入口到项目总目录的 `run_lidar.sh`，文件使用 LF 换行。它与定位入口 `run_loc.sh` 分开运行。

```bash
#!/usr/bin/env bash
set -euo pipefail
source "$HOME/.config/sany/workspace.bash"
set +u
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
exec ros2 launch livox_ros_driver2_core sany_lidar.launch.py \
  vehicle_config:="${SANY_VEHICLE_CONFIG:-/opt/conf/vehicle_config.yaml}"
```

在终端一运行并保持终端打开：

```bash
source "$HOME/.config/sany/workspace.bash"
bash "${SANY_WS}/run_lidar.sh"
```

自动入口按车辆文件选择三台或四台。旧车显式入口 `master_lidar.launch.py` 只包含前、左、右三台；`slave_lidar.launch.py` 继续在 slave 独立启动后雷达，旧车定位不使用该后雷达。若驱动已由其他终端或服务启动，先确认进程和工作区来源，避免重复绑定设备端口。

# 新车：主 Orin 同时启动四台雷达

四台雷达都接入 master 的新车使用 `master_4lidar.launch.py`，启动前、左、右、后四个独立驱动进程。前、左、右沿用上表配置；后雷达使用 `MID360s_config_rear_master.json`，master 接收网口配置为 `192.168.4.100`，后雷达 IP 为 `192.168.4.143`。后雷达的具体网卡名以新车实机为准，主机侧 UDP 端口仍为 56104–56504。

重新构建驱动后，在加载同一工作区环境的终端运行：

```bash
source "$HOME/.config/sany/workspace.bash"
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ros2 launch livox_ros_driver2_core master_4lidar.launch.py
```

也可将新车 `run_lidar.sh` 的最后一行改为 `exec ros2 launch livox_ros_driver2_core master_4lidar.launch.py`。四雷达入口已经包含原 master 的三路驱动，不要与 `master_lidar.launch.py` 或 `slave_lidar.launch.py` 同时运行。

四路点云保持 `sensor_msgs/msg/PointCloud2`、10 Hz 和原有 IP 后缀话题；四路 IMU 和心跳话题也保持原名。后雷达输出为 `/livox/lidar_192_168_4_143`、`/livox/imu_192_168_4_143` 和 `/diagnostics/heartbeat/livox_rear`，心跳 node_id 为 12。检查前三路时沿用下节命令，并补查后雷达：

```bash
ros2 topic hz /livox/lidar_192_168_4_143
ros2 topic hz /livox/imu_192_168_4_143
ros2 topic echo /diagnostics/heartbeat/livox_rear --once --qos-reliability best_effort
```

本入口只启动雷达驱动。在线定位的雷达布局仍由定位配置决定，启动四路驱动不会自动让定位使用后雷达。原 `slave_lidar.launch.py` 默认继续使用 `MID360s_config_rear.json` 中的 `192.168.4.101`。

# 输入验收与第二步定位

终端二加载相同环境，并逐条观察频率，每条结束时按 `Ctrl-C`：

```bash
source "$HOME/.config/sany/workspace.bash"
set +u
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ros2 topic info /livox/lidar_192_168_3_184
ros2 topic info /livox/lidar_192_168_1_108
ros2 topic info /livox/lidar_192_168_2_133
ros2 topic hz /livox/lidar_192_168_3_184
ros2 topic hz /livox/lidar_192_168_1_108
ros2 topic hz /livox/lidar_192_168_2_133
ros2 topic hz /livox/imu_192_168_3_184
ros2 topic echo /livox/imu_192_168_3_184 --once --qos-reliability best_effort --field header
```

三路点云类型均为 `sensor_msgs/msg/PointCloud2`，当前 Launch 配置为 10 Hz、`livox_frame`。2026-10-08 恢复后实测三路各 10 Hz，三路 IMU 各 200 Hz；这是该次现场结果，不能代替其他设备的测量。

Topic 列表可能因为订阅者存在而显示 Topic，`topic info` 应检查 Publisher 数量；最终还要收到实际消息。定位只使用前雷达 IMU，其他 IMU 可用于诊断。CAN 开启时先确认 CAN bridge 正常，再运行：

```bash
bash "${SANY_WS}/run_loc.sh"
```

停止时先结束定位，再结束雷达驱动。退出异常的证据与边界见 @ref livox_troubleshooting 。

# 配置与授时准备

修改 `config/MID360s_config_*.json` 或 Launch 后重新构建，并核对运行日志打印的安装树配置路径。主机网卡 IP 必须等于 JSON 的 `host_ip`，雷达必须在对应网段。

首次校时应在 PTP 对外授时和雷达采集前完成；运行期间避免时钟大幅回退。2026-10-08 已发现 RTC 本地时间配置和提前启动的 PTP，系统配置修复尚未执行，详见 @ref livox_incident_20261008 。
