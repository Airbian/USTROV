# ustrov_gz_plugins

USTROV 的 Gazebo Sim 插件集合，迁移自上游 `hippo_gz_plugins`。

系统插件包括：`barometer`、`buoyancy`、`hydrodynamics`、`kinematic_control`、`odometry`、`pose`、`range_sensor` 和 `thruster`；另包含把测距消息转换为 ROS 2 消息的 `range_sensor_bridge` 组件。

所有模型中的插件别名均使用 `ustrov_gz_plugins::*`。测距组件依赖本地 `ustrov_control_msgs`，不再依赖 `hippo_msgs`。
