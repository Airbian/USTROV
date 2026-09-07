#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <ustrov_control_msgs/msg/actuator_controls.hpp>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ustrov_core {

/**
 * @brief 将 USTROV 混控结果安全地转发给 PX4 的直接电机控制接口。
 *
 * 节点负责完整的解锁流程。解锁前必须已经收到有效混控命令和 PX4 状态；进入
 * Offboard 的过程中只发送零推力；PX4 同时报告 ARMED 和 OFFBOARD 后才转发
 * 真实推力。延续 PanorAUV 的稳定逻辑，VehicleStatus 只保存状态，不作为通信
 * 心跳。混控命令超时或 PX4 明确报告异常状态时，节点发送 NaN 并请求上锁。
 */
class Px4ActuatorBridge : public rclcpp::Node {
public:
  explicit Px4ActuatorBridge(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  static constexpr std::size_t kThrusterCount = 8;
  static constexpr std::size_t kPx4MotorCount = 12;

  // 从静止到解锁，再回到安全静止状态的完整流程。
  enum class State {
    kIdle,
    kPrestream,
    kWaitingForOffboard,
    kWaitingForArmed,
    kActive,
    kDisarming,
  };

  void DeclareAndReadParameters();
  void ValidateParameters();
  void CreateRosInterfaces();

  void OnThrusterCommand(
      const ustrov_control_msgs::msg::ActuatorControls::SharedPtr msg);
  void OnVehicleStatus(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void
  OnVehicleCommandAck(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg);
  void OnArmService(const std_srvs::srv::SetBool::Request::SharedPtr request,
                    std_srvs::srv::SetBool::Response::SharedPtr response);
  void OnStopService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                     std_srvs::srv::Trigger::Response::SharedPtr response);
  void OnOutputTimer();

  void StartArmSequence();
  void StartDisarming(const std::string &reason);
  void AbortArmSequence(const std::string &reason);
  void TransitionTo(State next_state, const std::string &reason);

  void PublishOffboardHeartbeat();
  void PublishMotorOutput(bool use_live_command);
  void PublishVehicleCommand(uint32_t command, float param1 = 0.0F,
                             float param2 = 0.0F);
  void RequestOffboardMode();
  void RequestArm();
  void RequestDisarm();

  bool HasFreshThrusterCommand() const;
  bool HasVehicleStatus() const;
  bool Px4IsArmed() const;
  bool Px4IsOffboard() const;
  bool Px4IsHealthy() const;
  uint64_t TimestampMicros() const;
  std::string Px4Topic(const std::string &suffix) const;
  static const char *StateName(State state);
  static const char *AckResultName(uint8_t result);

  std::string thruster_command_topic_;
  std::string px4_topic_prefix_;
  std::string vehicle_status_topic_;
  std::string vehicle_command_ack_topic_;
  double output_rate_hz_{50.0};
  std::chrono::milliseconds command_timeout_{300};
  std::chrono::milliseconds prestream_duration_{1200};
  std::chrono::milliseconds request_retry_period_{1000};
  std::chrono::milliseconds arm_sequence_timeout_{10000};
  bool require_fresh_command_to_arm_{true};
  bool auto_disarm_on_command_timeout_{true};
  uint16_t reversible_flags_{0x00FF};
  uint8_t target_system_{1};
  uint8_t target_component_{1};
  uint8_t source_system_{1};
  uint8_t source_component_{1};
  std::array<std::size_t, kThrusterCount> motor_input_indices_{};
  std::array<double, kThrusterCount> motor_signs_{};

  State state_{State::kIdle};
  std::array<double, kThrusterCount> latest_command_{};
  // 两个 received 标志用于区分“值为零”和“从未收到过消息”。
  bool command_received_{false};
  bool status_received_{false};
  px4_msgs::msg::VehicleStatus latest_status_{};
  rclcpp::Time last_command_time_;
  rclcpp::Time last_status_time_;
  rclcpp::Time state_entered_time_;
  rclcpp::Time last_request_time_;

  rclcpp::Subscription<ustrov_control_msgs::msg::ActuatorControls>::SharedPtr
      thruster_command_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr output_timer_;
};

} // namespace ustrov_core
