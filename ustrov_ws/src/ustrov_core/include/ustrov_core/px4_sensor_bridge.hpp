#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <px4_msgs/msg/sensor_baro.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

namespace ustrov_core {

/**
 * @brief 将 PX4 原始惯性与气压数据转换为标准 ROS 2 传感器消息。
 *
 * PX4 的机体系是 FRD（前、右、下），ROS 机器人常用 FLU（前、左、上）。
 * 本节点负责坐标转换、时间戳转换和协方差填充，不参与状态估计，也不向 PX4
 * 发送任何控制命令。
 */
class Px4SensorBridge : public rclcpp::Node {
public:
  explicit Px4SensorBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void DeclareAndReadParameters();
  void ValidateParameters();
  void CreateRosInterfaces();

  void OnSensorCombined(const px4_msgs::msg::SensorCombined::SharedPtr msg);
  void OnSensorBaro(const px4_msgs::msg::SensorBaro::SharedPtr msg);

  std::string Px4Topic(const std::string &suffix) const;
  rclcpp::Time Px4StampToRos(uint64_t timestamp_us);

  std::string px4_topic_prefix_;
  std::string sensor_combined_topic_;
  std::string sensor_baro_topic_;
  std::string imu_topic_;
  std::string pressure_topic_;
  std::string temperature_topic_;
  std::string imu_frame_id_;
  std::string pressure_frame_id_;

  double angular_velocity_stddev_{};
  double linear_acceleration_stddev_{};
  double pressure_stddev_pa_{};
  double temperature_stddev_c_{};

  bool time_offset_initialized_{false};
  int64_t px4_to_ros_offset_ns_{0};
  uint64_t latest_px4_timestamp_us_{0};

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr
      sensor_combined_sub_;
  rclcpp::Subscription<px4_msgs::msg::SensorBaro>::SharedPtr sensor_baro_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
};

} // namespace ustrov_core
