#pragma once
#include <ustrov_state_estimation/ekf.h>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ustrov_state_estimation_msgs/msg/estimator_innovation.hpp>
#include <ustrov_state_estimation_msgs/msg/estimator_sensor_bias.hpp>
#include <ustrov_state_estimation_msgs/msg/estimator_state.hpp>

struct EstimatorInnovation {
  uint64_t time_us;
  uint64_t time_sample_us;
  double vision_position[3];
  double baro_vertical_position;
  double heading;
};

class Estimator final : public rclcpp::Node {
 public:
  Estimator();
  void InitPublisher();
  //////////////////////////////////////////////////////////////////////////////
  // message callbacks
  //////////////////////////////////////////////////////////////////////////////
  void OnImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void OnBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg);
  void OnVision(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  void BaroUpdate();
  void VisionUpdate();

  void PublishAttitude(const rclcpp::Time &stamp);
  void PublishInnovations(const rclcpp::Time &stamp);
  void PublishPose(const rclcpp::Time &stamp);
  void PublishDelayedPose(const rclcpp::Time &stamp);
  void PublishSensorBias(const rclcpp::Time &stamp);
  void PublishState(const rclcpp::Time &stamp);
  void PublishVelocity(const rclcpp::Time &stamp);

  void ResetImuWatchdog() {
    imu_watchdog_.reset();
    imu_timed_out = false;
  }
  bool IsImuTimedOut() { return imu_timed_out; }
  void OnImuWatchdog();
  void Run();

 private:
  //////////////////////////////////////////////////////////////////////////////
  // Publisher
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Publisher<ustrov_state_estimation_msgs::msg::EstimatorInnovation>::SharedPtr
      innovation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      delayed_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr
      attitude_pub_;
  rclcpp::Publisher<ustrov_state_estimation_msgs::msg::EstimatorSensorBias>::SharedPtr
      sensor_bias_pub_;
  rclcpp::Publisher<ustrov_state_estimation_msgs::msg::EstimatorState>::SharedPtr
      state_pub_;

  //////////////////////////////////////////////////////////////////////////////
  // Subscriber
  //////////////////////////////////////////////////////////////////////////////
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      vision_sub_;
  rclcpp::TimerBase::SharedPtr imu_watchdog_;
  bool imu_timed_out{false};
  uint64_t imu_time_last_us;
  Ekf ekf_;
  bool baro_updated_{false};
  bool vision_updated_{false};
  BaroSample baro_sample_;
  Eigen::Vector3d gyro_bias_published_;
  Eigen::Vector3d accel_bias_published_;
  VisionSample vision_sample_;
  struct EstimatorParams {
    // 水的密度，淡水约 1000 kg/m^3，海水约 1025 kg/m^3。
    double water_density = 1000.0;
    // 启动时用于计算水面压力零点的样本数。
    int64_t surface_calibration_samples = 30;
  } params_;

  // 水压计零点只在启动阶段计算一次。标定完成前不向 EKF 送入深度。
  double surface_pressure_sum_{0.0};
  double pressure_at_surface_{0.0};
  int64_t surface_pressure_sample_count_{0};
  bool surface_pressure_calibrated_{false};
};
