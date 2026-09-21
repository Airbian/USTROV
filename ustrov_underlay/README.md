# ustrov_underlay

这个目录预留给主工作区的基础 ROS 2 源码依赖。它们应先于 `ustrov_ws`
构建，并通过 `install/setup.bash` 提供给主工作区。

当前纳入仓库管理的 underlay 包为：

| 包 | 固定 commit | 用途 |
|---|---|---|
| `px4_msgs` | `392e831c1f659429ca83902e66820d7094591410` | PX4 uXRCE-DDS 消息定义 |

`px4_msgs` 通过仓库根目录的 `.gitmodules` 固定为 Git submodule。克隆
USTROV 时需要同时初始化 submodule。不能直接换用任意版本的 `px4_msgs`，
它必须与实机 PX4 固件使用的消息定义一致。

Micro XRCE-DDS Agent 也适合放在 `ustrov_underlay/src` 中进行版本锁定。只应
纳入它的源码 submodule，不应提交 `build/`、`install/`、`log/` 等编译产物。
树莓派当前已经单独安装 Agent v2.4.3，后续可将同一版本加入此 underlay。

`apriltag_ros` 当前没有被 USTROV 运行链使用，因此暂不作为 submodule 管理；
等接入相机和 AprilTag 视觉定位时再加入。

预期目录结构：

```text
ustrov_underlay/
├── src/
│   ├── px4_msgs/
│   └── Micro-XRCE-DDS-Agent/  # 后续可加入
├── build/       # Git 忽略
├── install/     # Git 忽略
└── log/         # Git 忽略
```

首次克隆及构建：

```bash
git clone --recurse-submodules https://github.com/Airbian/USTROV.git
cd USTROV/ustrov_underlay
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
