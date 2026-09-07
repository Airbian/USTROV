# ustrov_underlay

这个目录预留给主工作区的基础 ROS 2 源码依赖。它们应先于 `ustrov_ws`
构建，并通过 `install/setup.bash` 提供给主工作区。

当前开发容器使用的 underlay 包为：

| 包 | 当前 package.xml 版本 | 用途 |
|---|---:|---|
| `px4_msgs` | `2.0.1` | PX4 uXRCE-DDS 消息定义 |
| `apriltag_ros` | `3.1.2` | AprilTag 视觉定位接口 |

目前只预留目录和版本说明，依赖源码尚未提交。正式加入前应记录准确的上游
仓库 URL 和 commit；尤其不能直接使用任意版本的 `px4_msgs`，它必须与实机
PX4 固件所使用的消息定义一致。

预期目录结构：

```text
ustrov_underlay/
├── src/
│   ├── px4_msgs/
│   └── apriltag_ros/
├── build/       # Git 忽略
├── install/     # Git 忽略
└── log/         # Git 忽略
```
