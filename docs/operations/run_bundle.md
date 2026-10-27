@page livox_run_bundle 雷达运行数据包

# 统一入口

参考定位的根入口、独立 runs 目录、输入检查及进程组退出方式，雷达每次启动创建一个数据包，不默认录制点云 bag，不新增 topic。构建并 source 新安装空间后，将 @ref livox_sany_deployment 中的根 `run_lidar.sh` 更新为包装入口。仅更新源码、仅重编而不修改旧根脚本不会自动启用。

```bash
source "$HOME/.config/sany/workspace.bash"
source /opt/ros/humble/setup.bash
source "$LIGHTNING_LM_INSTALL_SETUP"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
# 主、从 Orin 使用相同命令；hostname 自动区分角色
ros2 run livox_ros_driver2_core run_lidar_bundle.py --role auto \
  --output-root "${SANY_WS}/runs" --vehicle-config /opt/conf/vehicle_config.yaml
```

`--role` 默认 auto：`ubuntuM` / `sany-03-master` → master，`ubuntuS` / `sany-03-slave` → slave，忽略大小写及域名后缀。未知 hostname 拒绝启动，开发环境可显式指定 master/slave。角色由主机身份确定，主机再按车辆 LIDAR 选择三/四台，从机仅允许 MASTER AND SLAVE 的后雷达。

安装包附带统一根脚本；主从分别加载本机安装空间后执行（先备份已有根脚本）：

```bash
install -m 755 "$(ros2 pkg prefix livox_ros_driver2_core)/lib/livox_ros_driver2_core/run_lidar.sh" \
  "${SANY_WS}/run_lidar.sh"
bash "${SANY_WS}/run_lidar.sh"
```

脚本优先加载 `~/.config/sany/workspace.bash`，缺失时使用 `$HOME/Sany` 和其 `lightning_lm_ws/install/setup.bash`；不会改 hostname。两端都必须部署，不能只升级 master。

两端需要配套的新驱动、公共消息及正确的本机安装环境。数据检查在接收主机上运行，后雷达不自动加入主机定位。DDS、网卡、授时环境由部署配置提供，包装器不修改。运行前停止旧驱动，避免同一设备端口冲突；同一输出根/角色有文件锁，但不能阻止其他目录或直接 launch 的旧进程。

# 数据目录与查询

```text
runs/lidar_YYYYMMDD_HHMMSS_ffffff_master/
  run_metadata.json
  events.jsonl
  config/vehicle_config.yaml
  config/config/                 # 安装空间雷达 JSON 快照
  config/launch_ROS2/            # 安装空间 launch 快照
  config/<pid>_*                 # 节点启动时生成的参数文件
  logs/driver.log                # SDK/stdout/stderr 与 launch 输出
  logs/driver.log.1 ... .5       # 滚动日志
  logs/ros/                     # 本次 ROS_LOG_DIR
  logs/checks.log
  checks/startup_check.json
  checks/identity.json
  checks/device_state.json
  checks/result.json
  checks/failure_state.json      # 异常退出时的设备状态或查询错误
```

```bash
RUN="$(readlink -f "${SANY_WS}/runs/latest_lidar_master")"
tail -F "$RUN/logs/driver.log"
cat "$RUN/run_metadata.json"
cat "$RUN/checks/result.json"
# 从机将 latest_lidar_master 换成 latest_lidar_slave
```

终端打印唯一运行目录，`latest_lidar_master/slave` 原子更新。配置快照便于追溯，实际 launch 仍使用本次安装空间；运行期间不要修改安装文件。元数据记录安装路径、构建时 Git revision（dirty/unknown 显式保留）、ROS 环境白名单、初始配置哈希、各节点退出码和人工停止信号。不会整份导出环境变量。安装目录不含 Git 时，revision 为 unknown，不伪造提交。

# 短时检查与失败语义

启动后独立检查进程订阅所有预期雷达点云和 IMU 20 秒，保存计数、接收频率、空点云、Header 递增和时间年龄。通过条件为每路至少 3 条、末条接收年龄小于 1 秒、无空点云/不递增 Header，末条 Header 年龄在 -0.1 至 1 秒之间。频率作为测量结果保存，不将短窗口当长期丢帧验收。

之后用标准参数服务查询 SN（5 秒窗口），并用只读 Livox `0x0101` 查询每台 SN、工作状态、HMS。原始响应、返回码或超时分别保存；不支持的字段不当成 0。数据、身份或状态未知均不能通过总检查；HMS Error/Fatal 不通过，Warning 原样保留，不等于全部诊断通过。整个检查结束后退出，不常驻订阅或持续查询设备。

启动检查失败时保留驱动运行以便排查，`startup_check=failed` 写入元数据；现场不能继续启动定位。节点退出事件由 ROS Launch 的 OnProcessExit 获取，不依赖扫描日志字符串。节点非零退出会结束本次 launch，并记录具体退出码；即使 launch 本身返回 0，本次也判为 failed。异常结束再做一次只读设备快照；无响应也保留原因。

终端 Ctrl-C/SIGTERM 转发到独立 launch 进程组，25 秒后仍未退出则强杀并标记 `forced_kill`。检查进程也会回收。正常人工停止记 stopped；提前中断导致检查未完成仍记 failed（证据不足）；缺失节点退出事件、非零退出或检查失败都不能成为正常记录。主包装器 SIGKILL/主机断电无法执行收尾，元数据可能保留 running，应解释为未完整结束，不自动删。

# 容量与保留

`driver.log` 单文件约 10 MiB，保留 5 个滚动副本。ROS 自身日志并非逐文件轮转，整个运行目录默认超过 512 MiB 或磁盘可用不足 256 MiB 时请求停止，并标记 failed；每 5 秒抽查，允许短时超额。可通过 `--max-run-mb` 调整（至少 64）。启动前要求至少 1 GiB 空闲。

启动前自动清理本机、同一输出根、带本工具 owner 标记、已完整结束且通过检查的正常目录，仅保留最近 20 个历史正常包；新运行另计。`--keep-normal` 可调整。失败、未结束、外部目录和目录符号链接不清理。异常包可能长期积累，需要人工归档；工具不会为了腾空间删除故障证据。

# 验证边界

离线集成测试使用真实 ROS LaunchService 和合成进程，覆盖正常 Ctrl-C、节点 SIGSEGV、launch 成功不能掩盖节点失败，以及保留策略。设备采样检查和状态查询应在维护窗口实测。包装器记录退出故障，不修复 @ref livox_restart_test_20261027 中的驱动析构崩溃，不自动软重启雷达。

2026-10-27 独立目录只读验证：主 Orin 现有三雷达的数据、SN 参数、设备状态全部通过；从 Orin 旧驱动 `296e7f0` 的后雷达数据和设备状态通过，但身份参数缺失，工具正确记录 `identity_ok=false` 和 `invalid_response`，没有将其误判为全部通过。证据在两端 `/home/nvidia/Sany/lidar_bundle_validation_20261027/checks/`。本轮没有替换现场入口、安装新驱动或重启雷达；完整包装入口的进程管理验证来自离线 ROS 集成测试，不能写成已完成现场整套部署。

WSL Ubuntu 22.04 / ROS Humble 独立 build/install 验证目录为 `/tmp/livox_bundle_20261027`：4 个包构建安装成功，8 个功能 CTest（包含本功能的 4 个测试用例）通过，生成后的 25 页手册链接检查通过；定位仓库 11 项接口回归、62 页文档检查通过。新增 Python 文件单独检查 flake8。全仓库 CTest 中 copyright、cpplint、flake8、lint_cmake、pep257、uncrustify 六项 lint 未通过；旧基线 `c4d405e` 单独运行相同六项也全部失败。本次没有整库清理历史版权/格式规则，功能测试通过不等于全仓库 lint 通过。
