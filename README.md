# USTROV

USTROV 是基于 ROS 2 Jazzy、PX4 和 Gazebo 的水下机器人软件工作区。

## 仓库结构

```text
USTROV/
├── ustrov_ws/
│   └── src/                 # USTROV 自研与已迁移的 ROS 2 包
└── ustrov_underlay/
    └── src/                 # 预留给 px4_msgs 等基础依赖
```

`build/`、`install/`、`log/` 是 colcon 的本地生成目录，不纳入 Git。

## 当前 ROS 2 包

- `ustrov_bringup`：实机基础链统一启动入口
- `ustrov_common`：通用工具和 TF 发布
- `ustrov_control`：推进器混控
- `ustrov_control_msgs`：控制链自定义消息
- `ustrov_core`：PX4 执行器与传感器桥
- `ustrov_gz_plugins`：Gazebo 水动力及传感器插件
- `ustrov_remote_control`：手柄与键盘控制
- `ustrov_sim`：模型、世界和仿真 launch
- `ustrov_state_estimation`：IMU、压力与视觉融合 EKF
- `ustrov_state_estimation_msgs`：状态估计自定义消息

## 构建顺序

先准备并构建 `ustrov_underlay`，再构建主工作区：

```bash
source /opt/ros/jazzy/setup.bash

cd ~/USTROV/ustrov_underlay
colcon build --symlink-install
source install/setup.bash

cd ~/USTROV/ustrov_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

underlay 的具体依赖版本见
[`ustrov_underlay/README.md`](ustrov_underlay/README.md)。PX4 固件与
`px4_msgs` 必须保持消息版本兼容。

## 实机基础链

```bash
ros2 launch ustrov_bringup core.launch.py
```

首次进行推进器测试前，必须拆除螺旋桨或断开动力电源。
