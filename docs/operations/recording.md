@page livox_recording 录包脚本与工作流选择

# 选择采集入口

| 入口 | 数据与依赖 | 边界 |
|---|---|---|
| `record_3lidars_mcap.sh` / 安装别名 `record_lidars_mcap.sh` | 实际发现的完整点云+IMU Topic 对 | 脚本候选含前、左、右、后；名称中的 3 不限制为三台 |
| `record_lidars_cameras_mcap.sh` | 雷达、IMU 与相机采集 | 按脚本配置确认传感器和独立相机工作区 |
| 包内 `record_compressed_sensors_mcap.sh` | 四路点云 Zstd + 六路相机 H.265 | 已弃用兼容入口；仅启动雷达压缩器，要求相机 H.265 已发布，使用一个 recorder |
| 定位 `run_loc.sh` | 项目正式定位入口 | 当前固定关闭定位 MCAP，不由以上脚本自动启用 |

# 原始雷达采集

```bash
ros2 run livox_ros_driver2_core record_lidars_mcap.sh \
  30 /data/rosbag/livox_raw_test
```

第一个参数为正整数秒，默认 600；第二个为输出目录，默认按日期在 `/data/rosbag` 下创建。脚本先检查 MCAP 插件与安装树的 QoS 文件，按 30 s 发现窗口轮询完整点云+IMU Topic 对；外部查询没有单次超时，实际等待可能延长。没有发现任何完整 Topic 对时退出。

发现 Topic 对不能证明收到消息。录包前先完成 @ref livox_sany_deployment 的输入检查，之后以 `ros2 bag info` 核对每路实际数量。脚本按路录制再合并，正常中断需等待写盘与合并收尾；完整源码见 @ref livox_source_examples 。

# 脚本为何只录到了两台雷达？ {#livox_record_discovery_reading}

看候选集如何成为实际录制集：

@snippet{lineno} record_3lidars_mcap.sh livox-record-topic-pair

等待结束后，脚本只拒绝零路：

@snippet{lineno} record_3lidars_mcap.sh livox-record-partial-ready

<b>例子：</b>前、左各有点云和 IMU Topic，右仅有点云、后未出现，则 `active_indices` 包含两路。等待窗口结束后可以启动两路 recorder；脚本不会因缺少右/后而判定失败。候选为四台，也不按车辆配置自动要求某三台齐全。

<b>边界与停止点：</b>这里只检查 ROS 图中的 Topic 名称，不检查类型、发布者数量或消息是否持续到达；开始录制后也不动态加入新雷达。先核对打印的角色，再在 bag 内确认必需角色、类型、消息数、时间覆盖与缺口。读到非零判断即可停止，完整脚本：@ref record_3lidars_mcap.sh "原始录包源码"。

# 显示保存成功，是否已经确认所有 recorder 正常退出？ {#livox_record_completion_reading}

关闭阶段给各 recorder 发 SIGINT 并等待：

@snippet{lineno} record_3lidars_mcap.sh livox-record-stop-wait

<b>关键语句：</b>`wait ... || true` 忽略子进程退出码，且没有等待超时。源码会等待进程结束，但不能据此保证任意异常下都完成刷盘。随后单独判断合并命令：

@snippet{lineno} record_3lidars_mcap.sh livox-record-merge-failure

<b>例子：</b>某 recorder 中途退出，只留下较短但可读的 bag；如果后续转换成功，脚本仍可能打印合并成功。反过来，转换失败会退出 1，并保留原始分路 bag 供排查。脚本启动约两秒后检查进程，录制等待循环没有持续检查每个 recorder 的存活。

<b>边界与停止点：</b>“合并命令成功”和“所有预期数据完整”是两项验收。比较分路及合并包的 Topic、数量和起止时间，并抽样回放；本轮只核对脚本实现，没有执行真实录包。读到错误码处理即可停止。

# 压缩工作流

PointCloud2 压缩只处理 `data` 字节，保留 Header 与布局，解压默认输出 `/decompressed` 后缀避免环路。具体映射、参数和消息定义见 @ref livox_pointcloud_zstd 。

六相机四雷达的压缩录制入口见 @ref livox_compressed_recording 。它的容量、CPU 负载与验收口径需单独测量，不能沿用原始三雷达定位录包的估算。
