# ustrov_remote_control

本包由 Hippo 上游 `remote_control` 直接迁移，用 Xbox 风格手柄生成 USTROV
通用 mixer 所需的 `thrust_setpoint` 和 `torque_setpoint`。

```text
手柄 → joy → joystick_mapper
                 ├── thrust_setpoint
                 └── torque_setpoint
                          ↓
                 actuator_mixer_node
                          ↓
                   thruster_command
```

## 启动

```bash
ros2 launch ustrov_remote_control joystick_control.launch.py
```

该 launch 同时启动 ROS `joy` 驱动和手柄映射组件。默认读取设备 0，以 20 Hz
重复发布手柄状态。

### 键盘控制

```bash
ros2 launch ustrov_remote_control keyboard_control.launch.py
```

键盘节点发布与 Xbox 手柄相同布局的 `sensor_msgs/Joy`，然后继续复用原来的
`joystick_mapper`，因此其后续 thrust/torque、mixer 和 ustrov_core 链路完全相同。

| 按键 | 功能 | 虚拟 Joy 轴 |
|---|---|---|
| `W` / `S` | 正 / 负前后推力 `Fx` | 左摇杆上下 |
| `A` / `D` | 正 / 负偏航力矩 `Mz` | 左摇杆左右 |
| `↑` / `↓` | 正 / 负俯仰力矩 `My` | 右摇杆上下 |
| `←` / `→` | 正 / 负滚转力矩 `Mx` | 右摇杆左右 |
| `Space` | 所有轴立即归零 | 全部轴 |
| `Ctrl+C` | 关闭 launch | — |

终端本身没有按键释放事件，所以节点收到最后一个方向键 `0.6 s` 后会自动把该轴
归零。需要持续运动时应按住按键，让系统的键盘自动重复持续刷新命令。可用
`key_timeout:=0.8` 调整超时，用 `axis_value:=0.3` 限制键盘对应的最大摇杆量。

键盘节点必须从带 TTY 的交互式 Docker 终端启动；VS Code 的集成终端可以使用。

键盘 launch 会为 mapper 设置 `mapping.use_4dof:=true`，使用 Hippo 四推进器
构型对应的四自由度布局。四个推进器的推力轴都平行于机体 X 轴，所以 `W/S`
控制前进/后退 `Fx`，不是上浮/下潜 `Fz`。此模式只输出
`Fx、Mx、My、Mz`，并把物理上不能独立产生的 `Fy、Fz` 置零。

四路电机命令的典型符号关系为：

| 控制量 | motor 0 | motor 1 | motor 2 | motor 3 |
|---|---:|---:|---:|---:|
| `+Fx` 前进 | - | + | - | + |
| `+Mx` 滚转 | + | + | + | + |
| `+My` 俯仰 | - | + | + | - |
| `+Mz` 偏航 | + | + | - | - |

这里的正负号是电机命令，不是推进器的实际推力方向。推进器的旋转方向和桨叶
方向不同，因此 `+Fx` 对应的四路命令不是全部同号。

手柄 launch 不启用 `mapping.use_4dof`，因此仍保持上游原始映射。

## 上游手柄原始轴映射

| 手柄轴 | 输出 |
|---|---|
| 左摇杆上下 | `thrust_x` |
| 左摇杆左右 | `thrust_y` |
| 右摇杆上下 | `thrust_z` |
| 右摇杆左右 | `torque_z` |

当前 Hippo/USTROV mixer 只使用 `thrust_x` 和 `torque_x/y/z`，而上游原始手柄
模式将 `torque_x/y` 固定为零。因此 `joystick_control.launch.py` 当前实际可用的
控制是前进/后退与偏航。需要让实体手柄也使用完整的 Hippo 四自由度映射时，
应在对应 launch 中启用 `mapping.use_4dof`。

本节点不负责 Arm/Disarm。解锁仍必须显式调用 `ustrov_core` 服务。手柄断开后
`joy` 停止发布，mixer 和 ustrov_core 的看门狗负责归零与上锁。

## 相比 Hippo 上游的修改

- 包和 C++ 命名空间改为 `ustrov_remote_control`。
- `hippo_control_msgs` 改为 `ustrov_control_msgs`。
- `hippo_common` 改为 `ustrov_common`。
- launch 的 `vehicle_name` 默认留空，`use_sim_time` 默认 `false`。
- 删除 `hippo_msgs/NewtonGripperCommand`、夹爪发布器和 A/X 按钮夹爪逻辑。
- 保留原始摇杆轴编号、增益、20 Hz 自动重复和 thrust/torque 输出逻辑。
- 增加键盘输入节点以及 Hippo 四自由度 `Fx/Mx/My/Mz` 映射模式。
