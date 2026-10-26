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

# 从结论核对源码

关键路径采用“问题 → 真实短片段 → 阅读提示 → 输入例子 → 适用边界与停止点 → 完整 API/源码”的组织方式。片段直接使用 `@snippet{lineno}`，在真实 C++、Python 或 Shell 源文件中用成对、单独一行的 `// [稳定标识]` 或 `# [稳定标识]` 注释圈定；手写正文不维护固定行号，不复制一份伪源码。

`check_links.py` 会拒绝不存在或重名不明确的源文件、缺失/多余标记与空片段。`prepare_html.py` 显示生成时的仓库提交、未提交修改状态与 UTC 时间，并为导航资源添加版本。该来源是源码工作区，不代表车辆已安装的驱动版本。

短片段中的自动符号链接可能把局部变量跳到无关同名对象，生成后只保留其代码、行号与高亮；正文显式 API 链接以及完整源码/example 页仍保留导航。评审既要检查链接存在，也要核对命名空间与实际源文件是否正确。

运行 `python3 -m unittest discover -s docs/maintenance -p test_documentation_tools.py` 验证片段检查、来源标识和 HTML 处理。修改文档构建流程时核对独立 docs 与 `BUILD_DOCS=ON` 两个入口；源码标记修改要核对 C++ 非注释 token、Python AST 或 Shell 有效语句未变。生成文档通过不代表雷达、授时或录包通过实机验收。

新增或重命名 Launch、配置、脚本时同步 @ref livox_source_examples 。生成 HTML、构建缓存和日志不提交；上层 SANY 部署文档保留启动、验收与阅读入口，不复制整套开发手册。

# 新增专题的归属

沿用定位手册规则：先补充现有页；独立操作方法放在 `operations/`，故障证据放在 `operations/incidents/`。首页只保留已有 `guide/` 章节；事故与单次任务不得直接追加到首页。专题由所属章节唯一 `@subpage` 收录，跨页引用用 `@ref`。不在仓库根目录或 `docs/` 根目录追加任务报告。新增脚本同步源码注册与用途入口；安装/验证状态与历史恢复事实分开记录。HTML 检查必须核验真实导航树与源文件父节点一致。
