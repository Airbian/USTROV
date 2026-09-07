# ustrov_control

`ustrov_control` 包含 USTROV 的控制器和推力分配算法。当前实机首先使用与
Hippo 同构的通用 `mixer`：

```text
thrust_setpoint + torque_setpoint
              ↓
      actuator_mixer_node
              ↓
       thruster_command
              ↓
          ustrov_core
```

## USTROV mixer

启动默认物理配置：

```bash
ros2 launch ustrov_control node_actuator_mixer.launch.py
```

软件检查矩阵时使用归一化配置：

```bash
ros2 launch ustrov_control node_actuator_mixer.launch.py \
  mixer_path:=$(ros2 pkg prefix ustrov_control)/share/ustrov_control/config/actuator_mixer/ustrov_normalized_default.yaml
```

输入和输出均为相对话题，可以通过 `vehicle_name` 放入车辆命名空间。默认不使用
命名空间，与 `/thruster_command` 上的 `ustrov_core` 直接连接。

## 配置文件

- `ustrov_normalized_default.yaml`：线性归一化模型，只用于软件和无桨检查。
- `ustrov_default.yaml`：暂时沿用 Hippo 推进器模型，后续用 USTROV 台架数据替换。

两份配置都只启用 Motor0–Motor3；Motor4–Motor7 固定输出零。当前支持
`torque_x`、`torque_y`、`torque_z` 和 `thrust_x`，不使用 `thrust_y/z`。

输入超过 300 ms 未更新时，mixer 只发布一次更新后的零命令，随后停止发布，
以便 `ustrov_core` 的独立命令看门狗继续超时并请求 PX4 上锁。
