@page livox_guide_pipeline 数据链路与接口

沿点云与 IMU 的两条发布路径阅读，并区分进程存活、心跳工作计数、实际点云和时间对齐四类证据。

- @subpage livox_data_pipeline "SDK 接收、组帧与 ROS 发布"
- @subpage livox_heartbeat "功能安全心跳与健康边界"

源码验证顺序：@ref livox_packet_routes_reading "SDK 分流" → @ref livox_frame_time_reading "组帧与点时间" → @ref livox_queue_publish_reading "整帧队列与发布"。修改线程生命周期前先核对 @ref livox_shutdown_reading "阻塞等待与退出"；解释健康状态时另查 @ref livox_heartbeat_work_reading "工作序列" 和 @ref livox_diagnostic_faults_reading "故障保持/恢复"。
