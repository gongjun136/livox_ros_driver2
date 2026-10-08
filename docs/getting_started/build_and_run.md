@page livox_build_and_run 统一工作区构建与基本运行

# 统一源码布局

```text
lightning_lm_ws/src/
├── lightning-lm/
├── common_msgs/                # 含 livox_ros_driver2 与心跳接口
├── cgi430_driver/
├── livox_ros_driver2/          # 本仓库，包名 livox_ros_driver2_core
└── Livox-SDK2/                 # 包名 livox_sdk2
```

定位仓库每次更新到所用分支最新提交，配套仓库依照该版本的 `workspace.repos` 获取。清单使用提交哈希时，`vcs import` 显示 `detached HEAD` 是正常版本固定行为；实际部署记录各仓库 `git rev-parse HEAD`。

# 构建

首次部署先按项目部署文档创建 `$HOME/.config/sany/workspace.bash`。`SANY_WS` 是项目总目录，`LIGHTNING_LM_WS` 是工作区，`LIGHTNING_LM_INSTALL_SETUP` 指向统一工作区的 `install/local_setup.bash`。

```bash
source "$HOME/.config/sany/workspace.bash"
cd "$LIGHTNING_LM_WS"
vcs import src < src/lightning-lm/workspace.repos
bash src/lightning-lm/scripts/build_workspace.sh
```

驱动单独重编仍从统一工作区根目录运行，并按需构建接口与 SDK 依赖：

```bash
source "$HOME/.config/sany/workspace.bash"
set +u
source /opt/ros/humble/setup.bash
cd "$LIGHTNING_LM_WS"
MAKEFLAGS=-j4 colcon build --executor sequential \
  --packages-up-to livox_ros_driver2_core \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$LIGHTNING_LM_INSTALL_SETUP"
ros2 pkg prefix livox_ros_driver2_core
ros2 pkg prefix livox_ros_driver2
```

核心依赖包括 PCL、系统 Zstd、`diagnostic_monitor_interfaces`、共享 Livox 消息和 SDK2。CMake 可从统一 `install/livox_sdk2`、显式 `LIVOX_SDK2_ROOT` 或 `/usr/local` 查找 SDK；运行中实际加载的库可从 `/proc/<pid>/maps` 核对。

独立驱动工作区的历史构建方式仍保留在根 README；SANY 的统一部署以本页布局为准。不要在同一工作区重复复制消息包，也不要同时 source 多套旧驱动安装树。

# 启动与检查

```bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
ros2 launch livox_ros_driver2_core master_lidar.launch.py
```

现场固定入口、网卡与 Topic 表见 @ref livox_sany_deployment 。包的文档源码随 `install(DIRECTORY docs ...)` 安装到 `share/livox_ros_driver2_core/docs`；生成 HTML 的方法见 @ref livox_documentation_rules 。
