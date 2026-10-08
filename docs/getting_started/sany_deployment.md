@page livox_sany_deployment 主 Orin 三雷达部署

# 网络与拓扑

当前定位只使用主 Orin 本地接入的前、左、右雷达。下表是仓库配置及 2026-10-08 `sany-03-master` 的现场核对结果；其他车辆先核对自己的网卡和 JSON。

| 雷达 | 网卡 | 主机 IP | 雷达 IP | 点云 Topic | IMU Topic |
|---|---|---|---|---|---|
| 前 | eth3 | 192.168.3.100 | 192.168.3.184 | `/livox/lidar_192_168_3_184` | `/livox/imu_192_168_3_184` |
| 左 | eth1 | 192.168.1.100 | 192.168.1.108 | `/livox/lidar_192_168_1_108` | `/livox/imu_192_168_1_108` |
| 右 | eth2 | 192.168.2.100 | 192.168.2.133 | `/livox/lidar_192_168_2_133` | `/livox/imu_192_168_2_133` |

主机运维/DDS 接口在该现场为 `eth10 / 192.168.44.100`。传感器 UDP 与 ROS DDS 是两条链路，分别检查它们的地址和端口。

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
exec ros2 launch livox_ros_driver2_core master_lidar.launch.py
```

在终端一运行并保持终端打开：

```bash
source "$HOME/.config/sany/workspace.bash"
bash "${SANY_WS}/run_lidar.sh"
```

`master_lidar.launch.py` 只包含前、左、右三台雷达；`slave_lidar.launch.py` 的后雷达属于独立采集拓扑，当前定位不启用。若驱动已由其他终端或服务启动，先确认进程和工作区来源，避免重复绑定设备端口。

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
