# ustrov_state_estimation

本包是从上游状态估计模块迁移而来的 EKF。估计器以 IMU 为主时钟，融合压力计高度和视觉位姿。

## 数据链路

1. 每帧 IMU 数据触发一次预测，积分姿态、速度和位置，同时预测状态协方差。
2. 压力计和视觉样本进入延迟缓冲区，在与 IMU 相同的历史时刻参与更新。
3. 创新门控剔除与预测差异过大的观测。
4. 卡尔曼更新修正历史状态；输出预测器再把修正平滑传播到当前时刻。

命名空间为 `uuv00` 时，输入为：

- `/uuv00/imu`：`sensor_msgs/msg/Imu`
- `/uuv00/pressure`：`sensor_msgs/msg/FluidPressure`
- `/uuv00/vision_pose`：`geometry_msgs/msg/PoseWithCovarianceStamped`

主要输出位于 `/uuv00/state_estimator/`，包括 `pose`、`velocity`、`attitude`、`state`、`innovation` 和 `sensor_bias`。

单独启动：

```bash
ros2 launch ustrov_state_estimation state_estimation.launch.py \
  vehicle_name:=uuv00 use_sim_time:=true \
  water_density:=1000.0 surface_calibration_samples:=30
```

## 水压计深度零点

估计器启动后会先平均 `surface_calibration_samples` 帧压力数据，将结果作为
`pressure_at_surface`。标定完成前不会把压力数据送入 EKF；日志出现
`Pressure zero calibration complete` 后才开始输出相对水面的深度。

标定时必须让机器人在水面保持静止。深度按照下面的关系计算：

```text
z = -(pressure - pressure_at_surface) / (water_density * 9.81)
```

ROS 的 Z 轴向上，因此下潜后的 Z 为负。`water_density` 的单位是 `kg/m^3`：
淡水可使用约 `1000.0`，海水可使用约 `1025.0`。每次重启估计器都会重新标零；
如果在水下重启，当前位置会被当作新的零深度。

在完整仿真中启用真实 EKF（替代 `fake_state_estimator`）：

```bash
ros2 launch ustrov_sim top_hippocampus_complete.launch.py \
  vehicle_name:=uuv00 start_gui:=false \
  fake_state_estimation:=false fake_vision:=true
```

## 验证

运行单元测试：

```bash
cd ~/USTROV/ustrov_ws
colcon build --packages-select ustrov_state_estimation --symlink-install
colcon test --packages-select ustrov_state_estimation
colcon test-result --verbose
```

测试覆盖：

- 静止时姿态四元数有效，速度和位置保持有界；
- 恒定角速度能正确积分为姿态变化；
- 视觉位置更新能够通过创新门控并收敛到新位置；
- 代码格式和基础静态检查。

仿真运行后可检查频率：

```bash
ros2 topic hz /uuv00/imu
ros2 topic hz /uuv00/pressure
ros2 topic hz /uuv00/vision_pose
ros2 topic hz /uuv00/state_estimator/pose
```

核对估计值与 Gazebo 真值：

```bash
ros2 topic echo /uuv00/state_estimator/pose --once --field pose.pose
ros2 topic echo /uuv00/ground_truth/odometry --once --field pose.pose
```

当前限制：`state_estimator/pose` 的 ROS 6x6 协方差字段仍为零。内部 EKF 使用的是四元数状态协方差，后续需要先将其正确变换为 ROS 所需的 `roll/pitch/yaw` 协方差，不能直接复制矩阵元素。
