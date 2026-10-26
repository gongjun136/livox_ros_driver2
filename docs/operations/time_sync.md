@page livox_time_sync 开机授时门槛与时钟跳变防护

# 修复对象与边界

适用于 SANY 主 Orin 的 `chrony.service`、`ptp_master_eth{1,2,3}.service` 和对应 `_sys.service`，沿用现有三路硬件 PTP、`linreg` 以及 `-O 0` 时间尺度。其他车型、软件 PTP、不同网卡/服务名必须先核对，不能套用安装脚本。2026-10-14 的现场证据见 @ref livox_incident_20261014 。

这是主机授时启动顺序修复，不是点云解码修改。ROS package 构建/安装只交付工具，不自动改 systemd 或重启传感器。异常已经传入设备时，仅修复主机时间不能保证恢复，仍须在维护窗口软重启雷达并实测数据。

# 一次 Chrony 正常就会放行 PTP 吗？ {#livox_ptp_gate_reading}

先看一次查询怎样判为有效：

@snippet{lineno} sany_ptp_guard.py livox-ptp-quality

这里检查参考源、时间新鲜度、剩余修正、offset、skew 与组合误差预算；它们是启动准入条件。外层还要求连续稳定时间：

@snippet{lineno} sany_ptp_guard.py livox-ptp-stability

<b>例子：</b>连续四秒满足 tracking 门限后，实时时钟相对单调时钟跳了 20 ms，该次 `good` 被置为假，`stable_since` 清空；后续需要重新连续满足默认五秒。一次查询通过或 Chrony 服务为 active 都不足以放行。实际等待还受轮询节拍、命令耗时和总超时影响，不保证恰好五秒完成。

<b>边界与停止点：</b>这是启动阶段的采样检查，不能保证运行期间永不跳时，也不能证明每台雷达与主机的时间差已合格。`--check` 只等待并报告，不改时钟；`--prepare` 的副作用要继续核对下面的顺序。完整脚本：@ref sany_ptp_guard.py "授时保护源码"。

# 何时初始化 PHC，是否顺便立即修改系统时间？ {#livox_ptp_prepare_reading}

@snippet{lineno} sany_ptp_guard.py livox-ptp-prepare

<b>按语句读：</b>先检查该网卡对应的两个 systemd 服务 `MainPID` 均为 0，再等待时间稳定；`chronyc makestep 1 0` 修改未来自动 step 策略，第二次等待通过后才执行 `phc_ctl ... set` 初始化 PHC。第一轮等待失败时尚未执行策略修改；第二轮失败时策略已经修改，但 PHC 尚未初始化，函数没有回滚前一步。

<b>边界与停止点：</b>服务 PID 检查只覆盖指定服务，不能证明不存在其它控制同一 PHC 的进程，也不构成从检查到写入之间的全局互斥。该方法依赖维护流程和 systemd 启动顺序；不能在驱动运行时把它当作通用校时命令。读到 `phc_ctl` 即可停止，实际安装步骤与回退见后文。

# 启动契约

1. PTP 启动前调用 `sany_ptp_guard.py --prepare ethN`。检查 Chrony 实际跟踪状态，而不只检查服务 active。
2. 连续至少 5 秒满足：UTC 不早于 2025 年、有效非本地参考源、stratum 1–15、Leap Normal、参考更新时间不超过 180 秒、剩余修正及最近偏差不超过 10 ms、skew 不超过 100 ppm、估算误差上界不超过 100 ms。实时时钟相对单调时钟出现超过 10 ms 跳变则重新计时。这些是启动保护门限，不是定位精度验收指标。
3. 使用 `chronyc makestep 1 0` 关闭后续自动跳时，再次等待稳定；这是带两个参数的策略命令，不执行立即校时。然后在该网卡 PTP 与 phc2sys 都没有运行时用 `phc_ctl ethN set` 初始化 PHC，才允许 ptp4l 对外授时。
4. phc2sys 必须在该路 PTP 启动后运行，设置 `-S 0 -F 0`，不再在首轮或运行期 step PHC。PTP 依赖 Chrony，phc2sys 依赖对应 PTP；上游服务停止时相应下游也停止。Chrony 重启后须重新启动 PTP，通过新一轮门槛；不复用本次启动的缓存标记。
5. 超时、查询异常、陈旧参考、未同步或仍有控制进程时失败退出。保留 PTP 原有 Restart 策略，后续重试重新检查。不因等不到时间而放行 1970 年或假同步时间。

Chrony 持久配置改为 `makestep 1.0 3`，只允许最初三次更新时间大幅校正；门槛通过时进一步关闭剩余自动 step 机会。RTC 在已确认同步后写为 UTC，不使用 `--adjust-system-clock`。RTC 是改善启动初值的措施，不替代 NTP 门槛，也不能证明 RTC 硬件/上游授时已无故障。

参考：[Chrony 4.2 策略命令](https://chrony-project.org/doc/4.2/chronyc.html#makestep)、[LinuxPTP phc2sys 参数](https://www.linuxptp.org/documentation/phc2sys/)。原有 `-O 0` 保持不变，UTC/TAI 口径如需改动应另做兼容验收。

# 维护窗口安装

确认车辆停稳并停止定位、雷达入口。以下命令在主 Orin 执行，先按部署配置加载工作区变量。停止授时会影响依赖此时间源的传感器；不要在驾驶任务期间执行。

```bash
source "$HOME/.config/sany/workspace.bash"
cd "$LIGHTNING_LM_WS/src/livox_ros_driver2"
python3 scripts/sany_ptp_guard.py --check --timeout 30
sudo systemctl stop ptp_master_eth{1,2,3}_sys.service ptp_master_eth{1,2,3}.service
sudo bash scripts/install_sany_ptp_guard.sh --install
sudo systemctl start ptp_master_eth{1,2,3}.service
systemctl status ptp_master_eth{1,2,3}.service ptp_master_eth{1,2,3}_sys.service
```

安装器要求六个服务已停止，否则拒绝更改；备份存放在输出的 `/var/backups/sany-ptp-guard.XXXXXXXX`。它使用独立 drop-in，不覆盖原始服务文件；不改变 NTP server、不重启 Chrony、不重启域控或雷达。若安装中某步失败，应保持服务停止，按备份回退后再启动，不跳过错误继续上线。成功安装后才启动 PTP；PTP 会拉起对应 phc2sys。修复前已异常的雷达需软重启，再启动驱动。

从当前 shell 启动的进程无法继承 sudo 凭据缓存；若自动化账户提示需要密码，由管理员在自己的交互终端执行上述明确命令，不将密码写入脚本或文档。

# 验证与回退

```bash
python3 test/test_sany_ptp_guard.py
journalctl -b -o short-monotonic --no-pager \
  -u chrony -u ptp_master_eth1 -u ptp_master_eth1_sys
chronyc tracking
timedatectl
```

验证三路点云约 10 Hz、IMU 约 200 Hz，并检查 Header 时间、设备工作状态与 HMS；不能只看 topic 名称。必须另做冷启动：先有 Chrony 有效时间和 PHC 初始化日志，后有 PTP 对外授时；断开初始上游时间源时 PTP 应保持不出授时，恢复上游后允许重试通过。测试环境可用模拟时间验证拒绝条件，禁止在生产机上主动把时钟改成 1970 年。

回退也必须在维护窗口停止上述六个服务：用安装器输出的备份恢复 `/etc/chrony/chrony.conf`；六个 `50-sany-time-guard.conf` 如有旧备份则恢复，否则只删除本次新增的同名文件；`/usr/local/sbin/sany-ptp-guard` 同理。根据备份 `adjtime` 的 UTC/LOCAL 状态执行 `timedatectl set-local-rtc 0/1`（不改系统时间），再 `systemctl daemon-reload`。Chrony 如需恢复原运行策略，只能在 PTP 停止时重启，然后核验有效时间再启动授时。旧配置允许跳时，回退不等于修复。

此修复不监视或拦截管理员 `date -s`、宿主机强制调时、PTP 网络选主变化或可信上游发错时间。运行期失联时 Chrony 继续保持/渐调，本门槛不是持续精度监控；已有雷达诊断与定位时间门限仍需生效。冷启动、上游恢复及长时间运行尚未完成时，不得宣称现场永久修复已验收。
