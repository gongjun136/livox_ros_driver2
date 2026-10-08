# Livox 雷达驱动部署与开发手册

这套手册覆盖 `livox_ros_driver2_core` 的部署、采集链路、接口和排障。它与 Lightning-LM 手册采用相同的 Markdown、Doxygen、CMake 文档生成方式，可从章节进入 C++ API 和带行号源码。

**章节目录**

1. @subpage livox_guide_overview "系统与构建"：仓库边界、工作区、包名和构建入口。
2. @subpage livox_guide_deployment "现场部署与启动"：主 Orin 三雷达、网络配置、两步启动与输入验收。
3. @subpage livox_guide_pipeline "数据链路与接口"：SDK 回调、组帧、ROS 发布、时间戳和心跳。
4. @subpage livox_guide_recording "压缩与采集"：原始录包、点云 Zstd 和多传感器压缩工作流。
5. @subpage livox_guide_diagnostics "授时与排障"：只有 IMU 无点云、DDS、设备状态与现场案例。
6. @subpage livox_guide_maintenance "文档维护"：生成网站、同步规则和源码示例索引。

**推荐阅读顺序**

现场部署先读 @ref livox_build_and_run "统一工作区构建" → @ref livox_sany_deployment "三雷达启动" → @ref livox_configuration "配置与接口"。
开发修改先读 @ref livox_architecture "架构" → @ref livox_data_pipeline "采集到发布" → @ref livox_heartbeat "心跳语义"。
点云异常先读 @ref livox_troubleshooting "排障路径"，用 @ref livox_incident_20261008 "2026-10-08 授时案例" 对照实际证据。

当前 SANY 定位使用主机本地的前、左、右三路原始 PointCloud2，后雷达与压缩采集保留独立用途。源码决定行为；现场性能和恢复结果仅对记录版本与运行环境成立。
