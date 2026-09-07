# ustrov_core

`ustrov_core` 是 USTROV 实机上的 ROS 2 ↔ PX4 硬件接口包。它包含两个节点：

- `px4_actuator_bridge`：接收 8 路归一化推力并负责 Offboard、Arm、Disarm
  和命令超时保护；
- `px4_sensor_bridge`：把 PX4 原始 IMU 和气压计数据转换为标准 ROS 2
  传感器消息。

这个包只负责 PX4 硬件边界上的数据转换，不负责姿态/深度控制、状态估计或
推力分配。

## 数据链路

```text
ustrov_control
  /thruster_command (8 x [-1, 1])
          │
          ▼
ustrov_core / px4_actuator_bridge
          ├── /fmu/in/offboard_control_mode
          ├── /fmu/in/actuator_motors
          └── /fmu/in/vehicle_command
                         │
                         ▼
MicroXRCEAgent ── TELEM 串口 ── PX4 ── PWM ── ESC ── 推进器
```

PX4 返回的 `VehicleStatus` 和 `VehicleCommandAck` 用于确认模式、解锁状态和命令
结果。

传感器链路为：

```text
PX4 /fmu/out/sensor_combined ── FRD → FLU ── /uuv00/imu
PX4 /fmu/out/sensor_baro ─────────────────── /uuv00/pressure
                                      └───── /uuv00/barometer_temperature
```

`vehicle_attitude` 和 `vehicle_odometry` 是 PX4 的估计结果，不是原始传感器。
它们不会被送入本地 EKF 的 `imu` 或 `vision_pose`，以免产生估计反馈闭环。

## 核心设计

### 1. 与 PanorAUV 保持相同的 50 Hz 输出

在解锁准备、等待确认和正常运行阶段，桥接器以 50 Hz 发布：

- `OffboardControlMode`：`direct_actuator=true`，作为 PX4 Offboard 心跳。
- `ActuatorMotors`：解锁准备阶段为 0，正常运行阶段为实际推力。

`IDLE` 状态不发送 Offboard 心跳，也不会自动解锁。

### 2. VehicleStatus 只表示状态

`VehicleStatus` 是低频状态快照，不作为 DDS 心跳，也不设置消息超时。桥接器
保存最近一次 PX4 状态，用它判断：

- 是否已经 Armed；
- 是否处于 Offboard；
- 是否进入 Failsafe。

本包不订阅 `TimesyncStatus`。DDS/Offboard 链路中断最终由 PX4 自身的
Offboard-loss failsafe 处理。

### 3. 推力命令使用独立看门狗

`/thruster_command` 必须持续更新。默认超过 300 ms 没有新命令时，桥接器会：

1. 立即停止转发旧推力；
2. 发布全通道 `NaN`；
3. 向 PX4 发送 Disarm；
4. 每秒重试，直到 PX4 报告 Disarmed。

50 Hz 控制频率下，300 ms 约等于连续丢失 15 帧，既能快速停机，也能容忍少量
调度抖动。

### 4. 零推力与 NaN 的区别

- `0.0`：仍然控制该电机，但目标推力为零；用于解锁前预发送。
- `NaN`：放弃该执行器输出；用于上锁和急停。

输入消息任何一路出现 `NaN` 或 `Inf` 时，整帧都会被拒绝。若此时正在运行，
桥接器会直接进入上锁流程。

### 5. PX4 传感器坐标与时间

PX4 `SensorCombined` 使用 FRD 机体系（前、右、下）；`sensor_msgs/Imu` 使用
FLU 机体系（前、左、上）。转换规则为：

```text
x_ros =  x_px4
y_ros = -y_px4
z_ros = -z_px4
```

PX4 时间戳以飞控启动时刻为零点。传感器桥在第一帧计算 PX4 时间到 ROS 时间
的固定偏移，后续保留传感器原始采样间隔；检测到飞控重启后会自动重新标定。
`SensorCombined` 和 `SensorBaro` 不含测量协方差，YAML 中的标准差是初始保守
值，后续应通过实机静态采样统计后更新。

## ROS 接口

### 订阅

| 话题 | 类型 | 作用 |
|---|---|---|
| `/thruster_command` | `ustrov_control_msgs/msg/ActuatorControls` | 8 路归一化推力 |
| `/fmu/out/vehicle_status_v1` | `px4_msgs/msg/VehicleStatus` | Armed、Offboard、Failsafe 状态 |
| `/fmu/out/vehicle_command_ack` | `px4_msgs/msg/VehicleCommandAck` | PX4 命令确认 |

### 发布

| 话题 | 类型 | 作用 |
|---|---|---|
| `/fmu/in/offboard_control_mode` | `px4_msgs/msg/OffboardControlMode` | 50 Hz Offboard 心跳 |
| `/fmu/in/actuator_motors` | `px4_msgs/msg/ActuatorMotors` | 12 路 PX4 电机数组，前 8 路有效 |
| `/fmu/in/vehicle_command` | `px4_msgs/msg/VehicleCommand` | 切换模式、Arm、Disarm |

### 传感器桥接

| 输入 | 输出 | 输出类型 |
|---|---|---|
| `/fmu/out/sensor_combined` | `/uuv00/imu` | `sensor_msgs/msg/Imu` |
| `/fmu/out/sensor_baro` | `/uuv00/pressure` | `sensor_msgs/msg/FluidPressure` |
| `/fmu/out/sensor_baro` | `/uuv00/barometer_temperature` | `sensor_msgs/msg/Temperature` |

IMU 消息中的 `orientation_covariance[0]` 为 `-1`，表示该消息不提供姿态；姿态
必须由状态估计器计算。

### 服务

| 服务 | 类型 | 作用 |
|---|---|---|
| `/px4_actuator_bridge/arm` | `std_srvs/srv/SetBool` | `true` 解锁，`false` 上锁 |
| `/px4_actuator_bridge/stop` | `std_srvs/srv/Trigger` | 急停并请求上锁 |

使用车辆命名空间后，以上相对名称会自动加上对应前缀。

## 安全状态机

```text
IDLE
  │ arm=true，且 PX4 状态与推力命令有效
  ▼
PRESTREAM ── 50 Hz Offboard 心跳 + 零推力，至少 1.2 s
  ▼
WAITING_FOR_OFFBOARD ── 请求并等待 PX4 进入 Offboard
  ▼
WAITING_FOR_ARMED ── 请求并等待 PX4 确认 Armed
  ▼
ACTIVE ── 50 Hz 转发真实推力
  │
  ├── 命令超过 300 ms 未更新
  ├── PX4 报告 Failsafe
  ├── PX4 报告 Disarmed / 离开 Offboard
  └── stop 或 arm=false
  ▼
DISARMING ── 发布 NaN，重复发送 Disarm，确认后回到 IDLE
```

代码只发送普通 Arm 请求，不会绕过 PX4 预检。预检失败时应排查 QGroundControl
中的失败原因，不应使用强制解锁掩盖问题。

## 主要参数

参数文件：`config/px4_actuator_bridge.yaml`

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `output_rate_hz` | `50.0` | Offboard 和电机输出频率 |
| `command_timeout_ms` | `300` | 推力命令看门狗 |
| `prestream_duration_ms` | `1200` | 切入 Offboard 前的零推力预发送时间 |
| `request_retry_ms` | `1000` | 模式/解锁/上锁命令重试周期 |
| `arm_sequence_timeout_ms` | `10000` | 解锁流程最长等待时间 |
| `require_fresh_command_to_arm` | `true` | 没有新推力命令时拒绝解锁 |
| `auto_disarm_on_command_timeout` | `true` | 推力命令超时后自动上锁 |
| `motor_input_indices` | `[0..7]` | ROS 输入到 PX4 Motor1..8 的顺序映射 |
| `motor_signs` | 见 YAML | 每个推进器的方向修正 |
| `reversible_flags` | `255` | 前 8 路均允许双向推力 |

如果实际状态话题不是版本化名称，可修改 `vehicle_status_topic`，不要猜测后缀。

传感器桥参数文件：`config/px4_sensor_bridge.yaml`。其中可以修改输入话题、输出
话题、frame ID 和四项测量标准差。默认输出直接匹配
`ustrov_state_estimation` 的 `/uuv00/imu` 与 `/uuv00/pressure`。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_underlay/install/setup.bash

cd ~/ustrov_ws
colcon build --symlink-install --packages-select ustrov_core
source install/setup.bash
```

## 启动与测试

### 1. 启动 MicroXRCEAgent

```bash
MicroXRCEAgent serial --dev /dev/ttyPX4 -b 115200 -v 4
```

确认 PX4 输出话题存在：

```bash
ros2 topic echo /fmu/out/vehicle_status_v1 \
  px4_msgs/msg/VehicleStatus \
  --once --qos-reliability best_effort
```

### 2. 启动硬件桥接器

```bash
ros2 launch ustrov_core px4_hardware.launch.py
```

该 launch 会同时启动 `px4_actuator_bridge` 和 `px4_sensor_bridge`，但不会启动
MicroXRCEAgent。完整 `ustrov_bringup core.launch.py` 则会启动 Agent、两个桥接
节点和 mixer。

如果 PX4 使用 `uuv00` 命名空间：

```bash
ros2 launch ustrov_core px4_hardware.launch.py \
  vehicle_name:=uuv00 px4_topic_prefix:=/uuv00/fmu
```

### 3. 验证传感器桥

```bash
ros2 node info /px4_sensor_bridge
ros2 topic hz /uuv00/imu
ros2 topic hz /uuv00/pressure
ros2 topic echo /uuv00/imu --once
ros2 topic echo /uuv00/pressure --once
```

静止平放时，角速度应接近零；加速度模长应接近 `9.81 m/s²`。向机器人右侧
或下方转动时，可同时 echo `/fmu/out/sensor_combined`，确认 ROS 输出的 Y、Z
符号与 PX4 相反。

实机启动 EKF 时必须使用系统时间：

```bash
ros2 launch ustrov_state_estimation state_estimation.launch.py \
  vehicle_name:=uuv00 use_sim_time:=false
```

目前 PX4 只提供机载惯导估计，并没有独立的外部 `vision_pose`。因此这一步可
验证 IMU 预测与压力深度输入；要完成水平位置和航向观测，还需要接入
Qualisys、视觉定位或其他独立定位源。

### 4. 启动推力发布/控制节点

解锁前检查推力命令正在持续更新：

```bash
ros2 topic hz /thruster_command
```

### 5. 显式解锁和上锁

只有在拆桨或断开动力的条件下才能进行台架解锁测试：

```bash
ros2 service call /px4_actuator_bridge/arm \
  std_srvs/srv/SetBool "{data: true}"
```

正常上锁：

```bash
ros2 service call /px4_actuator_bridge/arm \
  std_srvs/srv/SetBool "{data: false}"
```

急停：

```bash
ros2 service call /px4_actuator_bridge/stop \
  std_srvs/srv/Trigger "{}"
```

## 已验证行为

- PX4 能稳定进入并保持 Armed + Offboard，不再因低频 `VehicleStatus` 误上锁。
- 停止推力发布程序后，实测约 318 ms 触发命令看门狗并发送 Disarm。
- PX4 返回 `latest_disarming_reason: 4`，表示由外部命令上锁。
- QGroundControl 约数秒后显示 `Not Ready / No offboard signal` 属于 Offboard
  readiness 更新，不代表推进器在这段时间仍有输出。

## 实机安全要求

- 首次测试必须拆除全部螺旋桨或断开推进器动力。
- 在 QGroundControl 中确认 Motor1..Motor8 对应正确 PWM 输出。
- 逐路核对 `motor_input_indices`、`motor_signs` 和实际旋转方向。
- PX4 固件与工作区 `px4_msgs` 必须使用兼容版本。
- 保留实体急停和断电手段，不能只依赖 ROS 服务。
- 修改频率、看门狗、映射或解锁逻辑后，必须重新进行无桨台架测试。
