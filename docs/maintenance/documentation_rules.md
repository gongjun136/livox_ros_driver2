@page livox_documentation_rules 文档生成与同步规则

# 独立生成

与 Lightning-LM 相同，从本仓库根目录运行，只需 CMake、Doxygen、Graphviz 与 Python 3，无需 ROS、SDK 安装或业务编译：

```bash
sudo apt-get install cmake doxygen graphviz python3
python3 docs/maintenance/check_links.py
cmake -S docs -B build_docs
cmake --build build_docs --target docs
python3 docs/maintenance/check_links.py --html build_docs/docs/html
python3 -m http.server 18770 --bind 127.0.0.1 \
  --directory build_docs/docs/html
```

首页为 `build_docs/docs/html/index.html`，服务运行期间访问 `http://127.0.0.1:18770/index.html`。修改 Markdown 或源码注释后重新生成并刷新，已有 HTTP 服务无需重启。构建扫描本仓库 `docs/` 与 `src/`，SDK API 的完整手册需在 SDK 仓库维护。

手写页 Doxygen 告警会使文档目标失败，既有源码注释告警留在 `build_docs/doxygen-warnings.log`。导航与 HTML 本地链接可用 Python 检查器核对。

# 随驱动构建

从已准备 ROS 与依赖的工作区根目录执行：

```bash
MAKEFLAGS=-j4 colcon build --executor sequential \
  --packages-up-to livox_ros_driver2_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=ON
```

驱动文档首页为 `build/livox_ros_driver2_core/docs/html/index.html`，后续可单独生成：

```bash
cmake --build build/livox_ros_driver2_core --target docs
python3 src/livox_ros_driver2/docs/maintenance/check_links.py \
  --html build/livox_ros_driver2_core/docs/html
```

`BUILD_DOCS` 默认为 OFF，已设置 ON 的构建缓存需显式设 OFF 才关闭。其他包若也支持这个选项，colcon 的同一参数会应用于它们，输出仍分别位于各包构建目录。

# 服务器阅读

服务器启动只监听回环的 HTTP 服务后，本机使用 SSH 隧道：

```bash
ssh -N -L 18771:127.0.0.1:18770 <服务器SSH别名>
```

浏览器访问 `http://127.0.0.1:18771/index.html`。不开放公网端口，文档 HTTP 服务不会启动雷达或定位。

# 更新对应关系

| 修改 | 同步章节 |
|---|---|
| CMake、package.xml、SDK 查找 | 构建页、架构页 |
| Launch、JSON、Topic、QoS | 部署页、配置页、项目部署文档 |
| SDK 回调、组帧、时间戳、队列与线程 | 数据链路页、排障页 |
| 心跳配置、work_seq、状态 | 心跳页、配置页 |
| 压缩节点、录包脚本 | 采集页及两篇原有压缩文档 |
| 退出/生命周期 | 数据链路页、排障页及已关闭故障记录 |

所有专题从 `index.md` 经 `guide/` 的 `@subpage` 唯一父节点导航；跨专题引用用 `@ref`。结论关联真实文件与符号，推断与未实施建议单独说明。现场记录保存提交、环境、症状、证据、改变的变量和实际恢复结果。

新增或重命名 Launch、配置、脚本时同步 @ref livox_source_examples 。生成 HTML、构建缓存和日志不提交；上层 SANY 部署文档保留启动、验收与阅读入口，不复制整套开发手册。
