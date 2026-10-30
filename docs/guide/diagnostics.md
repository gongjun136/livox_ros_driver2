@page livox_guide_diagnostics 授时与排障

先确定问题发生在雷达输出、UDP 接收、驱动组帧还是 DDS 消息传输，再决定是否需要恢复设备。涉及雷达重启时，现场必须允许中断传感器数据。

现场可执行 `bash ~/Sany/run_lidar.sh --diagnose [run_path] [--root ROOT] [--json] [--color auto|always|never] [--no-refresh]`。不指定路径时优先选择 `/proc` 中活跃 worker 的运行包，否则选择搜索根目录中的最新包；显式路径只离线读取该包。活跃包默认只读查询设备序列号、工作状态和 HMS 等小量状态，不订阅大点云 DDS、不新增常驻节点，也不写设备参数或重启设备；`--no-refresh` 强制只读运行包。

报告应区分启动数据检查与当前状态，并注明证据时间及缺失项；停止期间的 -11／-9 不能直接当作运行期崩溃。退出码 0 仅表示找到证据，不表示设备健康。定位自动诊断会以 `--json --no-refresh` 汇总当前上游雷达；显式检查历史定位包时不会拼入当前雷达状态。


- @subpage livox_troubleshooting "故障定位与授时检查"
- @subpage livox_run_bundle "雷达运行数据包与统一日志入口"
- @subpage livox_time_sync "开机授时门槛与时钟跳变防护"
- @subpage livox_incident_20261027 "sany-03-master 右雷达扫描模块异常与恢复"
- @subpage livox_restart_test_20261027 "双 Orin 四雷达驱动重启测试"
- @subpage livox_incident_20261014 "sany-03-master 开机时钟跳变与三雷达无数据"
- @subpage livox_incident_20261008 "sany-03-master 时间回退与点云恢复记录"

需要对照当前实现时，先看 @ref livox_ptp_gate_reading "PTP 启动所需证据"、@ref livox_ptp_prepare_reading "PHC 初始化顺序"；驱动端再看 @ref livox_frame_time_reading "组帧的时间来源与回退边界"、@ref livox_diagnostic_freshness_reading "设备诊断新鲜度"。历史事故页用于对照证据，不替代当前源码路径。
