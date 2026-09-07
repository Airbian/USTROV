
#include <ustrov_state_estimation/common.h>
#include <ustrov_state_estimation/ekf.h>
#include <ustrov_state_estimation/estimator.h>
#include <ustrov_state_estimation/interface.h>
#include <ustrov_state_estimation/util.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>

using std::placeholders::_1;

Estimator::Estimator() : Node("estimator_node") {
  imu_time_last_us = (uint64_t)now().nanoseconds() / 1000;
  InitPublisher();
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu", 10, std::bind(&Estimator::OnImu, this, _1));
  baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      "pressure", 10, std::bind(&Estimator::OnBaro, this, _1));
  vision_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "vision_pose", 10, std::bind(&Estimator::OnVision, this, _1));
  imu_watchdog_ =
      rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100),
                           std::bind(&Estimator::OnImuWatchdog, this));
}

void Estimator::InitPublisher() {
  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "~/pose", 10);
  delayed_pose_pub_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "~/delayed_pose", rclcpp::SystemDefaultsQoS());
  attitude_pub_ =
      create_publisher<geometry_msgs::msg::QuaternionStamped>("~/attitude", 10);
  sensor_bias_pub_ =
      create_publisher<ustrov_state_estimation_msgs::msg::EstimatorSensorBias>(
          "~/sensor_bias", 10);
  innovation_pub_ =
      create_publisher<ustrov_state_estimation_msgs::msg::EstimatorInnovation>(
          "~/innovation", 10);
  twist_pub_ =
      create_publisher<geometry_msgs::msg::TwistStamped>("~/velocity", 10);
  state_pub_ = create_publisher<ustrov_state_estimation_msgs::msg::EstimatorState>(
      "~/state", 10);
}

void Estimator::OnImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // Reset IMU Watchdog since we obviously got IMU data
  ResetImuWatchdog();
  ImuSample imu_sample_new;
  uint64_t dt_us;
  double dt;
  rclcpp::Time now_stamp = rclcpp::Time(msg->header.stamp);

  imu_sample_new.time_us =
      (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() * 1e-3);
  dt_us =
      Clip<uint64_t>(imu_sample_new.time_us - imu_time_last_us, 1000, 100000);
  imu_time_last_us = imu_sample_new.time_us;
  dt = dt_us * 1e-6;
  imu_time_last_us = imu_sample_new.time_us;
  imu_sample_new.delta_angle_dt = dt;
  imu_sample_new.delta_velocity_dt = dt;
  imu_sample_new.delta_angle =
      Eigen::Vector3d(msg->angular_velocity.x, msg->angular_velocity.y,
                      msg->angular_velocity.z) *
      dt;
  imu_sample_new.delta_velocity =
      Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y,
                      msg->linear_acceleration.z) *
      dt;

  ekf_.SetImuData(imu_sample_new);
  PublishAttitude(now_stamp);
  BaroUpdate();
  VisionUpdate();
  if (ekf_.Update()) {
    PublishState(now_stamp);
    PublishPose(now_stamp);
    PublishDelayedPose(now_stamp);
    PublishSensorBias(now_stamp);
    PublishInnovations(now_stamp);
    PublishVelocity(now_stamp);
  }
}

void Estimator::OnBaro(const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  baro_updated_ = true;
  // TODO: apply transformation from baro frame to body frame
  double pressure_at_bodyframe = msg->fluid_pressure;

  baro_sample_.height =
      -(pressure_at_bodyframe - params_.baro_atmo_pressure) * 1.0e-4 +
      params_.baro_sealevel_offset;
  baro_sample_.time_us =
      (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() * 1e-3);
  // ekf_.SetBaroData(sample);
}

void Estimator::OnVision(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  vision_updated_ = true;
  geometry_msgs::msg::Quaternion &q = msg->pose.pose.orientation;
  geometry_msgs::msg::Point &p = msg->pose.pose.position;
  vision_sample_.time_us =
      (uint64_t)(rclcpp::Time(msg->header.stamp).nanoseconds() * 1e-3);
  vision_sample_.orientation.w() = q.w;
  vision_sample_.orientation.x() = q.x;
  vision_sample_.orientation.y() = q.y;
  vision_sample_.orientation.z() = q.z;
  vision_sample_.position(0) = p.x;
  vision_sample_.position(1) = p.y;
  vision_sample_.position(2) = p.z;

  // PoseWithCovariance 使用 6x6 行优先矩阵：[x,y,z,roll,pitch,yaw]。
  // 给零或非法协方差设置保守下限，避免卡尔曼增益出现除零或随机值。
  constexpr double kMinPositionVariance = 1e-4;
  constexpr double kMinAngularVariance = 1e-4;
  constexpr int kPositionDiagonal[3] = {0, 7, 14};
  for (int i = 0; i < 3; ++i) {
    const double variance = msg->pose.covariance[kPositionDiagonal[i]];
    vision_sample_.position_variance(i) =
        std::isfinite(variance)
            ? std::max(variance, kMinPositionVariance)
            : kMinPositionVariance;
  }
  const double yaw_variance = msg->pose.covariance[35];
  vision_sample_.angular_variance =
      std::isfinite(yaw_variance)
          ? std::max(yaw_variance, kMinAngularVariance)
          : kMinAngularVariance;
}

void Estimator::BaroUpdate() {
  if (baro_updated_) {
    baro_updated_ = false;
    ekf_.SetBaroData(baro_sample_);
  }
}

void Estimator::VisionUpdate() {
  if (vision_updated_) {
    RCLCPP_DEBUG(get_logger(), "Vision updated.");
    vision_updated_ = false;
    ekf_.SetVisionData(vision_sample_);
  }
}

void Estimator::OnImuWatchdog() {
  // TODO: reset the estimator. IMU should never time out.
  imu_timed_out = true;
  RCLCPP_ERROR(
      get_logger(),
      "Imu data timed out! This should never happen during operation.");
}

void Estimator::PublishAttitude(const rclcpp::Time &stamp) {
  Eigen::Quaterniond q = ekf_.Orientation();
  geometry_msgs::msg::QuaternionStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;
  msg.quaternion.w = q.w();
  msg.quaternion.x = q.x();
  msg.quaternion.y = q.y();
  msg.quaternion.z = q.z();
  attitude_pub_->publish(msg);
}

void Estimator::PublishPose(const rclcpp::Time &stamp) {
  Eigen::Quaterniond q = ekf_.Orientation();
  Eigen::Vector3d p = ekf_.Position();
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;
  msg.pose.pose.position.x = p.x();
  msg.pose.pose.position.y = p.y();
  msg.pose.pose.position.z = p.z();
  msg.pose.pose.orientation.w = q.w();
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  // TODO: 将内部四元数协方差正确变换为 ROS 的 roll/pitch/yaw 协方差。
  // 在变换完成前保留零值，并只提示一次，避免高频输出污染日志。
  RCLCPP_WARN_ONCE(
      get_logger(),
      "Pose covariance publishing is not implemented; fields remain zero.");
  pose_pub_->publish(msg);
}

void Estimator::PublishDelayedPose(const rclcpp::Time &stamp) {
  StateVectord state = ekf_.StateAtFusionTime();
  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = "map";
  msg.header.stamp = stamp;
  msg.pose.pose.position.x = state(StateIndex::position_x);
  msg.pose.pose.position.y = state(StateIndex::position_y);
  msg.pose.pose.position.z = state(StateIndex::position_z);
  msg.pose.pose.orientation.w = state(StateIndex::qw);
  msg.pose.pose.orientation.x = state(StateIndex::qx);
  msg.pose.pose.orientation.y = state(StateIndex::qy);
  msg.pose.pose.orientation.z = state(StateIndex::qz);
  delayed_pose_pub_->publish(msg);
}

void Estimator::PublishSensorBias(const rclcpp::Time &stamp) {
  ustrov_state_estimation_msgs::msg::EstimatorSensorBias bias;
  bias.header.stamp = stamp;
  const Eigen::Vector3d gyro_bias{ekf_.GyroBias()};
  const Eigen::Vector3d accel_bias{ekf_.AccelerationBias()};
  if ((gyro_bias - gyro_bias_published_).norm() > 1e-3 ||
      (accel_bias - accel_bias_published_).norm() > 1e-3) {
    bias.gyro_bias[0] = gyro_bias.x();
    bias.gyro_bias[1] = gyro_bias.y();
    bias.gyro_bias[2] = gyro_bias.z();

    const Eigen::Vector3d gyro_bias_var{ekf_.GyroBiasVariance()};

    bias.gyro_bias_variance[0] = gyro_bias_var.x();
    bias.gyro_bias_variance[1] = gyro_bias_var.y();
    bias.gyro_bias_variance[2] = gyro_bias_var.z();

    bias.gyro_bias_valid = true;

    bias.acceleration_bias[0] = accel_bias.x();
    bias.acceleration_bias[1] = accel_bias.y();
    bias.acceleration_bias[2] = accel_bias.z();
    const Eigen::Vector3d accel_bias_var{ekf_.AccelerationBiasVariance()};
    bias.acceleration_bias_variance[0] = accel_bias_var.x();
    bias.acceleration_bias_variance[1] = accel_bias_var.y();
    bias.acceleration_bias_variance[2] = accel_bias_var.z();
    bias.acceleration_bias_valid =
        !ekf_.FaultStatusFlag().bad_acceleration_bias;

    gyro_bias_published_ = gyro_bias;
    accel_bias_published_ = accel_bias;

    sensor_bias_pub_->publish(bias);
  }
}

void Estimator::PublishInnovations(const rclcpp::Time &stamp) {
  ustrov_state_estimation_msgs::msg::EstimatorInnovation msg;
  msg.header.stamp = stamp;
  double position[3];
  ekf_.VisionPositionInnovation(position);
  msg.vision_position[0] = position[0];
  msg.vision_position[1] = position[1];
  msg.vision_position[2] = position[2];
  ekf_.BaroHeightInnovation(msg.baro_vertical_position);
  ekf_.HeadingInnovation(msg.heading);
  innovation_pub_->publish(msg);
}

void Estimator::PublishState(const rclcpp::Time &stamp) {
  ustrov_state_estimation_msgs::msg::EstimatorState msg;
  StateVectord state = ekf_.StateAtFusionTime();
  msg.header.stamp = stamp;
  msg.orientation.w = state(StateIndex::qw);
  msg.orientation.x = state(StateIndex::qx);
  msg.orientation.y = state(StateIndex::qy);
  msg.orientation.z = state(StateIndex::qz);
  msg.velocity.x = state(StateIndex::velocity_x);
  msg.velocity.y = state(StateIndex::velocity_y);
  msg.velocity.z = state(StateIndex::velocity_z);
  msg.position.x = state(StateIndex::position_x);
  msg.position.y = state(StateIndex::position_y);
  msg.position.z = state(StateIndex::position_z);
  msg.delta_angle_bias.x = state(StateIndex::delta_angle_bias_x);
  msg.delta_angle_bias.y = state(StateIndex::delta_angle_bias_y);
  msg.delta_angle_bias.z = state(StateIndex::delta_angle_bias_z);
  msg.delta_velocity_bias.x = state(StateIndex::delta_velocity_bias_x);
  msg.delta_velocity_bias.y = state(StateIndex::delta_velocity_bias_y);
  msg.delta_velocity_bias.z = state(StateIndex::delta_velocity_bias_z);
  state_pub_->publish(msg);
}

void Estimator::PublishVelocity(const rclcpp::Time &stamp) {
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = stamp;
  Eigen::Vector3d v = ekf_.Velocity();
  msg.twist.linear.x = v.x();
  msg.twist.linear.y = v.y();
  msg.twist.linear.z = v.z();
  twist_pub_->publish(msg);
}
void Estimator::Run() {}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Estimator>());
  rclcpp::shutdown();
  return 0;
}
