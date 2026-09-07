# ustrov_bringup

`ustrov_bringup` 提供 USTROV 实机每次启动都需要的基础控制链，不包含手柄、
键盘或任何自动解锁行为。

```text
MicroXRCEAgent
      │ /dev/ttyPX4 @ 115200
      ▼
PX4 DDS topics
      ├────► ustrov_core / px4_sensor_bridge
      │             ├── /uuv00/imu
      │             └── /uuv00/pressure
      │
      └────► ustrov_core / px4_actuator_bridge
                          ▲
                          │ thruster_command
ustrov_control / actuator_mixer_node
      ▲
      │ thrust_setpoint + torque_setpoint
手柄、键盘或自主控制（另行启动）
```

## 启动

确认 WSL USB 映射和 Docker 设备映射已经完成：

```bash
ls -l /dev/ttyPX4
```

然后在容器中运行：

```bash
cd ~/ustrov_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_underlay/install/setup.bash
source install/setup.bash

ros2 launch ustrov_bringup core.launch.py
```

默认等价于：

```bash
ros2 launch ustrov_bringup core.launch.py \
  start_agent:=true \
  agent_device:=/dev/ttyPX4 \
  agent_baudrate:=115200 \
  agent_verbose:=4 \
  px4_topic_prefix:=/fmu
```

如果已经在另一个终端或 Docker 服务中启动了 MicroXRCEAgent：

```bash
ros2 launch ustrov_bringup core.launch.py start_agent:=false
```

不要让两个 Agent 同时打开同一个串口。

如需覆盖底层参数文件，沿用两个子 launch 原有的参数名：

```bash
ros2 launch ustrov_bringup core.launch.py \
  config_file:=/path/to/px4_actuator_bridge.yaml \
  sensor_config_file:=/path/to/px4_sensor_bridge.yaml \
  mixer_path:=/path/to/ustrov_default.yaml
```

传感器桥默认输出 `/uuv00/imu`、`/uuv00/pressure` 和
`/uuv00/barometer_temperature`。它只读取 PX4，不发送控制命令。

## 手动控制另行启动

键盘：

```bash
ros2 launch ustrov_remote_control keyboard_control.launch.py axis_value:=0.3
```

手柄：

```bash
ros2 launch ustrov_remote_control joystick_control.launch.py
```

键盘和手柄不要同时运行，因为它们会发布相同的 setpoint 话题。

## 安全边界

- 本 launch 不调用 Arm 服务，启动后 PX4 仍保持 Disarmed。
- mixer 在没有输入时不会产生非零推力命令。
- 首次联合测试必须拆桨或断开推进器动力。
- Agent 串口错误不会绕过 `ustrov_core` 的状态检查和推力看门狗。
