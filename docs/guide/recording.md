@page livox_guide_recording 压缩与采集

正式定位直接消费原始 PointCloud2。Zstd 压缩、六相机四雷达采集属于独立工作流，启用采集前核对 Topic、存储空间、QoS 和实际增长率。

- @subpage livox_recording "录包脚本与工作流选择"
- @subpage livox_pointcloud_zstd "PointCloud2 无损压缩与解压"
- @subpage livox_compressed_recording "六相机四雷达压缩录制"

先核对 @ref livox_zstd_lossless_reading "序列化与无损压缩边界"、@ref livox_zstd_decode_reading "解码校验"，再读 @ref livox_record_discovery_reading "topic 发现" 和 @ref livox_record_completion_reading "录制退出"。多传感器流程的进程顺序与收尾见 @ref livox_compressed_entry_reading 和 @ref livox_compressed_flush_reading 。
