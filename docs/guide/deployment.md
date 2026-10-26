@page livox_guide_deployment 现场部署与启动

现场先启动雷达，再启动定位；雷达、定位与检查终端加载同一工作区及 DDS 配置。Topic 名称出现后还要检查实际消息、频率和 Header 时间。

- @subpage livox_sany_deployment "主 Orin 雷达部署"
- @subpage livox_configuration "网络、Launch 参数与消息契约"

先用 @ref livox_launch_layout_reading "VIN/LIDAR 选择" 确认启动范围，再用 @ref livox_role_binding_reading "心跳 ID 与角色" 核对每个进程绑定的设备。完成后再检查实际数据，不以启动日志代替收包。
