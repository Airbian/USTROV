#include "ustrov_core/px4_sensor_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>
#include <stdexcept>

using std::placeholders::_1;

namespace ustrov_core {

namespace {

constexpr int64_t kNanosecondsPerMicrosecond = 1000;
constexpr uint64_t kPx4ResetThresholdUs = 5'000'000;

bool IsFiniteVector(const std::array<float, 3> &value) {
  return std::all_of(value.begin(), value.end(),
                     [](float element) { return std::isfinite(element); });
}

void SetDiagonalCovariance(std::array<double, 9> &covariance,
                           double variance) {
  covariance.fill(0.0);
  covariance[0] = variance;
  covariance[4] = variance;
  covariance[8] = variance;
}

} // namespace

Px4SensorBridge::Px4SensorBridge(const rclcpp::NodeOptions &options)
    : Node("px4_sensor_bridge", options) {
  DeclareAndReadParameters();
  ValidateParameters();
  CreateRosInterfaces();
}

void Px4SensorBridge::DeclareAndReadParameters() {
  px4_topic_prefix_ = declare_parameter<std::string>("px4_topic_prefix", "/fmu");
  sensor_combined_topic_ =
      declare_parameter<std::string>("sensor_combined_topic", "");
  sensor_baro_topic_ = declare_parameter<std::string>("sensor_baro_topic", "");

  // 输出使用绝对话题名，与现有 uuv00 状态估计器接口直接对应。
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/uuv00/imu");
  pressure_topic_ =
      declare_parameter<std::string>("pressure_topic", "/uuv00/pressure");
  temperature_topic_ = declare_parameter<std::string>(
      "temperature_topic", "/uuv00/barometer_temperature");
  imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "base_link");
  pressure_frame_id_ =
      declare_parameter<std::string>("pressure_frame_id", "base_link");

  angular_velocity_stddev_ =
      declare_parameter<double>("angular_velocity_stddev", 0.02);
  linear_acceleration_stddev_ =
      declare_parameter<double>("linear_acceleration_stddev", 0.20);
  pressure_stddev_pa_ =
      declare_parameter<double>("pressure_stddev_pa", 100.0);
  temperature_stddev_c_ =
      declare_parameter<double>("temperature_stddev_c", 1.0);

  while (px4_topic_prefix_.size() > 1 && px4_topic_prefix_.back() == '/') {
    px4_topic_prefix_.pop_back();
  }
  if (sensor_combined_topic_.empty()) {
    sensor_combined_topic_ = Px4Topic("/out/sensor_combined");
  }
  if (sensor_baro_topic_.empty()) {
    sensor_baro_topic_ = Px4Topic("/out/sensor_baro");
  }
}

void Px4SensorBridge::ValidateParameters() {
  const auto require_nonnegative_finite = [](double value,
                                              const char *parameter_name) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(std::string(parameter_name) +
                                  " must be finite and >= 0");
    }
  };
  require_nonnegative_finite(angular_velocity_stddev_,
                             "angular_velocity_stddev");
  require_nonnegative_finite(linear_acceleration_stddev_,
                             "linear_acceleration_stddev");
  require_nonnegative_finite(pressure_stddev_pa_, "pressure_stddev_pa");
  require_nonnegative_finite(temperature_stddev_c_, "temperature_stddev_c");
}

void Px4SensorBridge::CreateRosInterfaces() {
  // PX4 输出采用 Best Effort；ROS 侧重新发布为 Reliable，兼容现有 EKF 订阅者。
  auto px4_qos = rclcpp::SensorDataQoS();
  px4_qos.keep_last(5);
  auto ros_qos = rclcpp::QoS(rclcpp::KeepLast(20));
  ros_qos.reliable().durability_volatile();

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, ros_qos);
  pressure_pub_ = create_publisher<sensor_msgs::msg::FluidPressure>(
      pressure_topic_, ros_qos);
  temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(
      temperature_topic_, ros_qos);

  sensor_combined_sub_ =
      create_subscription<px4_msgs::msg::SensorCombined>(
          sensor_combined_topic_, px4_qos,
          std::bind(&Px4SensorBridge::OnSensorCombined, this, _1));
  sensor_baro_sub_ = create_subscription<px4_msgs::msg::SensorBaro>(
      sensor_baro_topic_, px4_qos,
      std::bind(&Px4SensorBridge::OnSensorBaro, this, _1));

  RCLCPP_INFO(get_logger(), "PX4 IMU: %s -> %s",
              sensor_combined_topic_.c_str(), imu_topic_.c_str());
  RCLCPP_INFO(get_logger(), "PX4 barometer: %s -> %s, %s",
              sensor_baro_topic_.c_str(), pressure_topic_.c_str(),
              temperature_topic_.c_str());
}

void Px4SensorBridge::OnSensorCombined(
    const px4_msgs::msg::SensorCombined::SharedPtr msg) {
  if (!IsFiniteVector(msg->gyro_rad) ||
      !IsFiniteVector(msg->accelerometer_m_s2)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropped PX4 IMU sample containing NaN or Inf");
    return;
  }

  sensor_msgs::msg::Imu output;
  output.header.stamp = Px4StampToRos(msg->timestamp);
  output.header.frame_id = imu_frame_id_;

  // 180 度绕 X 轴等价于 FRD -> FLU：X 不变，Y/Z 取反。
  output.angular_velocity.x = msg->gyro_rad[0];
  output.angular_velocity.y = -msg->gyro_rad[1];
  output.angular_velocity.z = -msg->gyro_rad[2];
  output.linear_acceleration.x = msg->accelerometer_m_s2[0];
  output.linear_acceleration.y = -msg->accelerometer_m_s2[1];
  output.linear_acceleration.z = -msg->accelerometer_m_s2[2];

  // SensorCombined 不包含姿态；-1 明确告诉下游不要使用 orientation。
  output.orientation.w = 1.0;
  output.orientation_covariance.fill(0.0);
  output.orientation_covariance[0] = -1.0;
  SetDiagonalCovariance(output.angular_velocity_covariance,
                        angular_velocity_stddev_ * angular_velocity_stddev_);
  SetDiagonalCovariance(
      output.linear_acceleration_covariance,
      linear_acceleration_stddev_ * linear_acceleration_stddev_);
  imu_pub_->publish(output);
}

void Px4SensorBridge::OnSensorBaro(
    const px4_msgs::msg::SensorBaro::SharedPtr msg) {
  if (!std::isfinite(msg->pressure) || msg->pressure <= 0.0F ||
      !std::isfinite(msg->temperature)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Dropped invalid PX4 barometer sample");
    return;
  }

  const uint64_t sample_timestamp =
      msg->timestamp_sample == 0 ? msg->timestamp : msg->timestamp_sample;
  const auto stamp = Px4StampToRos(sample_timestamp);

  sensor_msgs::msg::FluidPressure pressure;
  pressure.header.stamp = stamp;
  pressure.header.frame_id = pressure_frame_id_;
  pressure.fluid_pressure = msg->pressure;
  pressure.variance = pressure_stddev_pa_ * pressure_stddev_pa_;
  pressure_pub_->publish(pressure);

  sensor_msgs::msg::Temperature temperature;
  temperature.header.stamp = stamp;
  temperature.header.frame_id = pressure_frame_id_;
  temperature.temperature = msg->temperature;
  temperature.variance = temperature_stddev_c_ * temperature_stddev_c_;
  temperature_pub_->publish(temperature);
}

std::string Px4SensorBridge::Px4Topic(const std::string &suffix) const {
  if (px4_topic_prefix_.empty() || px4_topic_prefix_ == "/") {
    return suffix;
  }
  return px4_topic_prefix_ + suffix;
}

rclcpp::Time Px4SensorBridge::Px4StampToRos(uint64_t timestamp_us) {
  const auto ros_now = now();
  if (timestamp_us == 0) {
    return ros_now;
  }

  // PX4 时间从飞控启动开始计数；首次收到数据时求固定偏移，以后保留传感器 dt。
  const bool px4_restarted =
      time_offset_initialized_ &&
      timestamp_us + kPx4ResetThresholdUs < latest_px4_timestamp_us_;
  if (!time_offset_initialized_ || px4_restarted) {
    px4_to_ros_offset_ns_ =
        ros_now.nanoseconds() -
        static_cast<int64_t>(timestamp_us) * kNanosecondsPerMicrosecond;
    time_offset_initialized_ = true;
    latest_px4_timestamp_us_ = timestamp_us;
    if (px4_restarted) {
      RCLCPP_WARN(get_logger(),
                  "PX4 timestamp reset detected; ROS time offset recalibrated");
    }
  } else {
    latest_px4_timestamp_us_ =
        std::max(latest_px4_timestamp_us_, timestamp_us);
  }

  const int64_t stamp_ns =
      px4_to_ros_offset_ns_ +
      static_cast<int64_t>(timestamp_us) * kNanosecondsPerMicrosecond;
  return rclcpp::Time(stamp_ns, get_clock()->get_clock_type());
}

} // namespace ustrov_core
