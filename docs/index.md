# Livox 雷达驱动部署与开发手册

这套手册覆盖 `livox_ros_driver2_core` 的部署、采集链路、接口和排障。它与 Lightning-LM 手册采用相同的 Markdown、Doxygen、CMake 文档生成方式，可从章节进入 C++ API 和带行号源码。

<b>章节目录</b>

1. @subpage livox_guide_overview "系统与构建"：仓库边界、工作区、包名和构建入口。
2. @subpage livox_guide_deployment "现场部署与启动"：主 Orin 三/四雷达、网络配置、两步启动与输入验收。
3. @subpage livox_guide_pipeline "数据链路与接口"：SDK 回调、组帧、ROS 发布、时间戳和心跳。
4. @subpage livox_guide_recording "压缩与采集"：原始录包、点云 Zstd 和多传感器压缩工作流。
5. @subpage livox_guide_diagnostics "授时与排障"：只有 IMU 无点云、DDS、设备状态与现场案例。
6. @subpage livox_guide_maintenance "文档维护"：生成网站、同步规则和源码示例索引。

<b>从问题进入源码</b>

每个入口都提供真实短片段、应读的条件或赋值、可手算例子、适用边界和完整源码链接。先核对一个问题，再沿需要的分支深入。

| 要确认的问题 | 短验证路径 |
|---|---|
| 为什么启动三台或四台，角色与心跳 ID 怎样对应？ | @ref livox_launch_layout_reading "车辆配置选择" → @ref livox_role_binding_reading "单设备角色绑定" |
| IMU 正常但点云没有，数据可能停在哪一层？ | @ref livox_packet_routes_reading "SDK 分流" → @ref livox_frame_time_reading "组帧与时间" → @ref livox_queue_publish_reading "队列到发布" |
| 心跳、Running、有效点云和诊断通过分别说明什么？ | @ref livox_heartbeat_work_reading "心跳与工作进度" → @ref livox_diagnostic_freshness_reading "有效性与新鲜度" |
| 发了退出请求，为何线程还可能等待？ | @ref livox_shutdown_reading "退出信号与阻塞等待" |
| 压缩是否修改点属性，解码失败会怎样？ | @ref livox_zstd_lossless_reading "压缩字节边界" → @ref livox_zstd_decode_reading "解码校验" |
| 录包发现了 topic，就能证明成功保存吗？ | @ref livox_record_discovery_reading "发现条件" → @ref livox_record_completion_reading "完成与退出" |
| Chrony 服务 active，为什么仍不能启动 PTP？ | @ref livox_ptp_gate_reading "有效授时证据" → @ref livox_ptp_prepare_reading "PHC 初始化顺序" |

<b>推荐阅读顺序</b>

现场部署先读 @ref livox_build_and_run "统一工作区构建" → @ref livox_sany_deployment "三/四雷达启动" → @ref livox_configuration "配置与接口"。
开发修改先读 @ref livox_architecture "架构" → @ref livox_data_pipeline "采集到发布" → @ref livox_heartbeat "心跳语义"。
点云异常先读 @ref livox_troubleshooting "排障路径"，用 @ref livox_incident_20261008 "2026-10-08 授时案例" 对照实际证据。

SANY 驱动按车辆配置选择三台或四台；定位实际使用哪些输入仍以同次定位配置为准。压缩采集是独立工作流。源码决定行为；现场性能和恢复结果仅对记录版本与运行环境成立。
