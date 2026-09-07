#include "ustrov_core/px4_actuator_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace ustrov_core {

Px4ActuatorBridge::Px4ActuatorBridge(const rclcpp::NodeOptions &options)
    : Node("px4_actuator_bridge", options),
      last_command_time_(0, 0, get_clock()->get_clock_type()),
      last_status_time_(0, 0, get_clock()->get_clock_type()),
      state_entered_time_(now()),
      last_request_time_(0, 0, get_clock()->get_clock_type()) {
  latest_command_.fill(0.0);
  // 先读取并校验参数，再创建 ROS 接口，避免带错误配置的节点开始运行。
  DeclareAndReadParameters();
  ValidateParameters();
  CreateRosInterfaces();

  RCLCPP_INFO(
      get_logger(),
      "PX4 actuator bridge ready. Input='%s', PX4 prefix='%s'. "
      "The vehicle will remain idle until '~/arm' is called with data=true.",
      thruster_command_topic_.c_str(), px4_topic_prefix_.c_str());
}

void Px4ActuatorBridge::DeclareAndReadParameters() {
  // 话题、频率和安全超时均可在 config/px4_actuator_bridge.yaml 中覆盖。
  thruster_command_topic_ = declare_parameter<std::string>(
      "thruster_command_topic", "thruster_command");
  px4_topic_prefix_ =
      declare_parameter<std::string>("px4_topic_prefix", "/fmu");
  vehicle_status_topic_ =
      declare_parameter<std::string>("vehicle_status_topic", "");
  vehicle_command_ack_topic_ =
      declare_parameter<std::string>("vehicle_command_ack_topic", "");
  output_rate_hz_ = declare_parameter<double>("output_rate_hz", 50.0);
  command_timeout_ = std::chrono::milliseconds(
      declare_parameter<int>("command_timeout_ms", 300));
  prestream_duration_ = std::chrono::milliseconds(
      declare_parameter<int>("prestream_duration_ms", 1200));
  request_retry_period_ = std::chrono::milliseconds(
      declare_parameter<int>("request_retry_ms", 1000));
  arm_sequence_timeout_ = std::chrono::milliseconds(
      declare_parameter<int>("arm_sequence_timeout_ms", 10000));
  require_fresh_command_to_arm_ =
      declare_parameter<bool>("require_fresh_command_to_arm", true);
  auto_disarm_on_command_timeout_ =
      declare_parameter<bool>("auto_disarm_on_command_timeout", true);
  reversible_flags_ =
      static_cast<uint16_t>(declare_parameter<int>("reversible_flags", 255));
  target_system_ =
      static_cast<uint8_t>(declare_parameter<int>("target_system", 1));
  target_component_ =
      static_cast<uint8_t>(declare_parameter<int>("target_component", 1));
  source_system_ =
      static_cast<uint8_t>(declare_parameter<int>("source_system", 1));
  source_component_ =
      static_cast<uint8_t>(declare_parameter<int>("source_component", 1));

  const auto input_indices = declare_parameter<std::vector<int64_t>>(
      "motor_input_indices", {0, 1, 2, 3, 4, 5, 6, 7});
  const auto signs = declare_parameter<std::vector<double>>(
      "motor_signs", {1.0, 1.0, 1.0, 1.0, -1.0, 1.0, 1.0, -1.0});

  if (input_indices.size() != kThrusterCount ||
      signs.size() != kThrusterCount) {
    throw std::invalid_argument("motor_input_indices and motor_signs must each "
                                "contain exactly 8 values");
  }
  std::array<bool, kThrusterCount> input_index_used{};
  for (std::size_t i = 0; i < kThrusterCount; ++i) {
    if (input_indices[i] < 0 ||
        input_indices[i] >= static_cast<int64_t>(kThrusterCount)) {
      throw std::invalid_argument(
          "motor_input_indices values must be in [0, 7]");
    }
    motor_input_indices_[i] = static_cast<std::size_t>(input_indices[i]);
    if (input_index_used[motor_input_indices_[i]]) {
      throw std::invalid_argument(
          "motor_input_indices must be a permutation of [0, 7]");
    }
    input_index_used[motor_input_indices_[i]] = true;
    motor_signs_[i] = signs[i];
  }
}

void Px4ActuatorBridge::ValidateParameters() {
  // 参数错误时直接拒绝启动，避免把错误通道或异常频率带到实机。
  if (output_rate_hz_ < 2.0 || output_rate_hz_ > 400.0) {
    throw std::invalid_argument("output_rate_hz must be in [2, 400]");
  }
  if (command_timeout_.count() <= 0 || prestream_duration_.count() < 1000 ||
      request_retry_period_.count() <= 0 ||
      arm_sequence_timeout_.count() <= prestream_duration_.count()) {
    throw std::invalid_argument(
        "Timeout parameters are invalid; prestream must be >=1000 ms and the "
        "arm sequence timeout must be longer than prestream");
  }
  for (const double sign : motor_signs_) {
    if (!std::isfinite(sign) || std::abs(sign) != 1.0) {
      throw std::invalid_argument(
          "Every motor_signs value must be +1.0 or -1.0");
    }
  }
  if (px4_topic_prefix_.empty()) {
    px4_topic_prefix_ = "/fmu";
  }
  if (px4_topic_prefix_.front() != '/') {
    px4_topic_prefix_.insert(px4_topic_prefix_.begin(), '/');
  }
  while (px4_topic_prefix_.size() > 1 && px4_topic_prefix_.back() == '/') {
    px4_topic_prefix_.pop_back();
  }
  if (vehicle_status_topic_.empty()) {
    vehicle_status_topic_ = Px4Topic("/out/vehicle_status");
  }
  if (vehicle_command_ack_topic_.empty()) {
    vehicle_command_ack_topic_ = Px4Topic("/out/vehicle_command_ack");
  }
}

void Px4ActuatorBridge::CreateRosInterfaces() {
  // PX4 DDS 话题使用 Best Effort；只保留最新样本，避免执行过期控制命令。
  auto px4_input_qos = rclcpp::QoS(rclcpp::KeepLast(1));
  px4_input_qos.best_effort().durability_volatile();
  auto px4_output_qos = rclcpp::SensorDataQoS();
  px4_output_qos.keep_last(5);
  auto mixer_qos = rclcpp::SensorDataQoS();
  mixer_qos.keep_last(1);

  motors_pub_ = create_publisher<px4_msgs::msg::ActuatorMotors>(
      Px4Topic("/in/actuator_motors"), px4_input_qos);
  offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      Px4Topic("/in/offboard_control_mode"), px4_input_qos);
  command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      Px4Topic("/in/vehicle_command"), px4_input_qos);

  thruster_command_sub_ =
      create_subscription<ustrov_control_msgs::msg::ActuatorControls>(
          thruster_command_topic_, mixer_qos,
          std::bind(&Px4ActuatorBridge::OnThrusterCommand, this, _1));
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      vehicle_status_topic_, px4_output_qos,
      std::bind(&Px4ActuatorBridge::OnVehicleStatus, this, _1));
  ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(
      vehicle_command_ack_topic_, px4_output_qos,
      std::bind(&Px4ActuatorBridge::OnVehicleCommandAck, this, _1));

  arm_service_ = create_service<std_srvs::srv::SetBool>(
      "~/arm", std::bind(&Px4ActuatorBridge::OnArmService, this, _1, _2));
  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop", std::bind(&Px4ActuatorBridge::OnStopService, this, _1, _2));

  const auto period = std::chrono::duration<double>(1.0 / output_rate_hz_);
  output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Px4ActuatorBridge::OnOutputTimer, this));
}

void Px4ActuatorBridge::OnThrusterCommand(
    const ustrov_control_msgs::msg::ActuatorControls::SharedPtr msg) {
  // 任一路出现 NaN/Inf 都拒绝整帧；运行中遇到异常值则立即上锁。
  for (std::size_t i = 0; i < kThrusterCount; ++i) {
    const double value = msg->control[i];
    if (!std::isfinite(value)) {
      command_received_ = false;
      latest_command_.fill(0.0);
      RCLCPP_ERROR(get_logger(),
                   "Rejected thruster_command: channel %zu is NaN or Inf", i);
      if (state_ != State::kIdle && state_ != State::kDisarming) {
        StartDisarming("non-finite thruster command received");
      }
      return;
    }
    latest_command_[i] = std::clamp(value, -1.0, 1.0);
  }
  command_received_ = true;
  last_command_time_ = now();
}

void Px4ActuatorBridge::OnVehicleStatus(
    const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
  // VehicleStatus 是状态快照而不是心跳：只缓存最新值，不设置新鲜度超时。
  latest_status_ = *msg;
  status_received_ = true;
  last_status_time_ = now();
}

void Px4ActuatorBridge::OnVehicleCommandAck(
    const px4_msgs::msg::VehicleCommandAck::SharedPtr msg) {
  if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE ||
      msg->command ==
          px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
    if (msg->result ==
        px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
      RCLCPP_INFO(get_logger(), "PX4 accepted command %u", msg->command);
    } else {
      RCLCPP_WARN(get_logger(), "PX4 command %u result: %s (%u)", msg->command,
                  AckResultName(msg->result), msg->result);
    }
  }
}

void Px4ActuatorBridge::OnArmService(
    const std_srvs::srv::SetBool::Request::SharedPtr request,
    std_srvs::srv::SetBool::Response::SharedPtr response) {
  if (!request->data) {
    // arm=false 与急停服务使用同一套安全上锁流程。
    StartDisarming("disarm requested by ROS service");
    response->success = true;
    response->message = "Disarm sequence started; motor outputs are NaN.";
    return;
  }

  if (state_ != State::kIdle) {
    response->success = false;
    response->message =
        std::string("Cannot start arm sequence while state is ") +
        StateName(state_);
    return;
  }
  if (!HasVehicleStatus()) {
    response->success = false;
    response->message =
        "No PX4 VehicleStatus received; check MicroXRCEAgent and TELEM.";
    return;
  }
  if (!Px4IsHealthy()) {
    response->success = false;
    response->message = "PX4 reports failsafe; refusing to arm.";
    return;
  }
  if (Px4IsArmed()) {
    response->success = false;
    response->message =
        "PX4 is already armed; refusing to take over unexpectedly.";
    return;
  }
  if (require_fresh_command_to_arm_ && !HasFreshThrusterCommand()) {
    response->success = false;
    response->message =
        "No fresh thruster_command; start the controller first.";
    return;
  }

  StartArmSequence();
  response->success = true;
  response->message =
      "Arm sequence started: zero prestream -> Offboard -> PX4 arm checks.";
}

void Px4ActuatorBridge::OnStopService(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  StartDisarming("emergency stop requested by ROS service");
  response->success = true;
  response->message =
      "Emergency stop active; NaN outputs and disarm requested.";
}

void Px4ActuatorBridge::OnOutputTimer() {
  const auto current_time = now();

  // 分别打印每项安全条件，便于区分命令超时、PX4 状态变化和 Failsafe。
  const auto log_guard_state = [this, &current_time](const char *context) {
    const auto age_ms = [&current_time](bool received,
                                        const rclcpp::Time &last_time) {
      if (!received) {
        return -1.0;
      }
      return static_cast<double>((current_time - last_time).nanoseconds()) /
             1.0e6;
    };
    const auto bool_text = [](bool value) { return value ? "true" : "false"; };

    const bool has_status = HasVehicleStatus();
    const bool fresh_command = HasFreshThrusterCommand();
    const bool healthy = Px4IsHealthy();
    const bool offboard = Px4IsOffboard();
    const bool armed = Px4IsArmed();

    RCLCPP_ERROR(
        get_logger(),
        "Guard failure in %s: has_status=%s "
        "status_age_ms=%.1f (informational), fresh_command=%s "
        "command_age_ms=%.1f/%.1f, healthy=%s, offboard=%s, armed=%s, "
        "failsafe=%s, preflight=%s, nav_state=%u, arming_state=%u, "
        "gcs_lost=%s",
        context, bool_text(has_status),
        age_ms(status_received_, last_status_time_),
        bool_text(fresh_command), age_ms(command_received_, last_command_time_),
        static_cast<double>(command_timeout_.count()), bool_text(healthy),
        bool_text(offboard), bool_text(armed),
        bool_text(status_received_ && latest_status_.failsafe),
        bool_text(status_received_ && latest_status_.pre_flight_checks_pass),
        status_received_ ? static_cast<unsigned>(latest_status_.nav_state) : 0U,
        status_received_ ? static_cast<unsigned>(latest_status_.arming_state)
                         : 0U,
        bool_text(status_received_ && latest_status_.gcs_connection_lost));
  };

  switch (state_) {
  case State::kIdle:
    // IDLE 不发送 Offboard 心跳，也不会自动解锁。
    return;

  case State::kPrestream:
    // 按 PX4 要求，切入 Offboard 前先持续发送至少 1 秒零推力。
    if (!HasVehicleStatus() || !Px4IsHealthy()) {
      log_guard_state("PRESTREAM status");
      AbortArmSequence("PX4 status is unavailable or entered failsafe");
      return;
    }
    if (require_fresh_command_to_arm_ && !HasFreshThrusterCommand()) {
      log_guard_state("PRESTREAM command");
      AbortArmSequence("thruster command timed out during prestream");
      return;
    }
    PublishOffboardHeartbeat();
    PublishMotorOutput(false);
    if ((current_time - state_entered_time_).nanoseconds() >=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            prestream_duration_)
            .count()) {
      RequestOffboardMode();
      TransitionTo(State::kWaitingForOffboard,
                   "zero setpoints prestreamed; Offboard requested");
    }
    return;

  case State::kWaitingForOffboard:
    // 等待 PX4 确认 Offboard；等待期间始终保持零推力。
    if (!HasVehicleStatus() || !Px4IsHealthy() ||
        (require_fresh_command_to_arm_ && !HasFreshThrusterCommand())) {
      log_guard_state("WAITING_FOR_OFFBOARD");
      AbortArmSequence(
          "status/command or healthy PX4 condition was lost");
      return;
    }
    PublishOffboardHeartbeat();
    PublishMotorOutput(false);
    if (Px4IsOffboard()) {
      RequestArm();
      TransitionTo(State::kWaitingForArmed,
                   "PX4 entered Offboard; arm requested");
    } else if ((current_time - last_request_time_).nanoseconds() >=
               std::chrono::duration_cast<std::chrono::nanoseconds>(
                   request_retry_period_)
                   .count()) {
      RequestOffboardMode();
    }
    break;

  case State::kWaitingForArmed:
    // 只有 PX4 明确报告 ARMED 后，才允许进入真实推力输出阶段。
    if (!HasVehicleStatus() || !Px4IsHealthy() || !Px4IsOffboard() ||
        (require_fresh_command_to_arm_ && !HasFreshThrusterCommand())) {
      log_guard_state("WAITING_FOR_ARMED");
      AbortArmSequence("PX4/status/command condition was lost while arming");
      return;
    }
    PublishOffboardHeartbeat();
    PublishMotorOutput(false);
    if (Px4IsArmed()) {
      TransitionTo(State::kActive,
                   "PX4 reports ARMED and OFFBOARD; live commands enabled");
    } else if ((current_time - last_request_time_).nanoseconds() >=
               std::chrono::duration_cast<std::chrono::nanoseconds>(
                   request_retry_period_)
                   .count()) {
      RequestArm();
    }
    break;

  case State::kActive:
    // ACTIVE 中每个 50 Hz 周期都检查 PX4 状态和 300 ms 命令看门狗。
    if (!HasVehicleStatus() || !Px4IsHealthy() || !Px4IsArmed() ||
        !Px4IsOffboard()) {
      log_guard_state("ACTIVE PX4 state");
      StartDisarming("PX4 reports failsafe, disarmed, or left Offboard");
      return;
    }
    if (!HasFreshThrusterCommand()) {
      log_guard_state("ACTIVE command watchdog");
      if (auto_disarm_on_command_timeout_) {
        StartDisarming("thruster_command watchdog timeout");
      } else {
        PublishOffboardHeartbeat();
        PublishMotorOutput(false);
      }
      return;
    }
    PublishOffboardHeartbeat();
    PublishMotorOutput(true);
    return;

  case State::kDisarming:
    // 持续发送 NaN，并周期性重发 Disarm，直到 PX4 确认已经上锁。
    PublishMotorOutput(false);
    if (HasVehicleStatus() && !Px4IsArmed()) {
      TransitionTo(State::kIdle, "PX4 reports DISARMED");
      return;
    }
    if ((current_time - last_request_time_).nanoseconds() >=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            request_retry_period_)
            .count()) {
      RequestDisarm();
    }
    return;
  }

  if (state_ == State::kWaitingForOffboard ||
      state_ == State::kWaitingForArmed) {
    if ((current_time - state_entered_time_).nanoseconds() >=
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            arm_sequence_timeout_)
            .count()) {
      AbortArmSequence("arm sequence timed out");
    }
  }
}

void Px4ActuatorBridge::StartArmSequence() {
  TransitionTo(State::kPrestream,
               "arm requested; streaming zero direct-actuator setpoints");
}

void Px4ActuatorBridge::StartDisarming(const std::string &reason) {
  // 先切换状态再发布，确保第一帧就是全通道 NaN，不会残留一帧零指令。
  TransitionTo(State::kDisarming, reason);
  PublishMotorOutput(false);
  RequestDisarm();
}

void Px4ActuatorBridge::AbortArmSequence(const std::string &reason) {
  // 即使本地状态可能滞后，也始终请求上锁，并等待 PX4 明确报告 DISARMED。
  StartDisarming(reason);
}

void Px4ActuatorBridge::TransitionTo(State next_state,
                                     const std::string &reason) {
  if (state_ == next_state) {
    return;
  }
  RCLCPP_WARN(get_logger(), "State %s -> %s: %s", StateName(state_),
              StateName(next_state), reason.c_str());
  state_ = next_state;
  state_entered_time_ = now();
}

void Px4ActuatorBridge::PublishOffboardHeartbeat() {
  // direct_actuator=true 表示绕过 PX4 内部控制器，直接使用 ActuatorMotors。
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = TimestampMicros();
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = true;
  offboard_mode_pub_->publish(msg);
}

void Px4ActuatorBridge::PublishMotorOutput(bool use_live_command) {
  px4_msgs::msg::ActuatorMotors msg{};
  msg.timestamp = TimestampMicros();
  msg.timestamp_sample = msg.timestamp;
  msg.reversible_flags = reversible_flags_;
  msg.control.fill(std::numeric_limits<float>::quiet_NaN());

  for (std::size_t motor = 0; motor < kThrusterCount; ++motor) {
    // 解锁准备阶段发送 0；ACTIVE 阶段才读取并映射真实混控值。
    double value = 0.0;
    if (use_live_command) {
      const auto input = motor_input_indices_[motor];
      value = latest_command_[input] * motor_signs_[motor];
    }
    msg.control[motor] = static_cast<float>(std::clamp(value, -1.0, 1.0));
  }

  // PX4 将 NaN 解释为“不控制/上锁输出”；因此 DISARMING 必须覆盖全部通道。
  if (state_ == State::kDisarming) {
    msg.control.fill(std::numeric_limits<float>::quiet_NaN());
  }
  motors_pub_->publish(msg);
}

void Px4ActuatorBridge::PublishVehicleCommand(uint32_t command, float param1,
                                              float param2) {
  // 模式切换、Arm 和 Disarm 都通过 PX4 VehicleCommand 发送。
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = TimestampMicros();
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = target_system_;
  msg.target_component = target_component_;
  msg.source_system = source_system_;
  msg.source_component = source_component_;
  msg.from_external = true;
  command_pub_->publish(msg);
  last_request_time_ = now();
}

void Px4ActuatorBridge::RequestOffboardMode() {
  PublishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                        1.0F, 6.0F);
  RCLCPP_INFO(get_logger(), "Requested PX4 Offboard mode");
}

void Px4ActuatorBridge::RequestArm() {
  PublishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F);
  RCLCPP_INFO(get_logger(),
              "Requested PX4 arm (normal preflight checks apply)");
}

void Px4ActuatorBridge::RequestDisarm() {
  PublishVehicleCommand(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0F);
  RCLCPP_WARN(get_logger(), "Requested PX4 disarm");
}

bool Px4ActuatorBridge::HasFreshThrusterCommand() const {
  if (!command_received_) {
    return false;
  }
  return (now() - last_command_time_).nanoseconds() <=
         std::chrono::duration_cast<std::chrono::nanoseconds>(command_timeout_)
             .count();
}

bool Px4ActuatorBridge::HasVehicleStatus() const { return status_received_; }

bool Px4ActuatorBridge::Px4IsArmed() const {
  return status_received_ &&
         latest_status_.arming_state ==
             px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
}

bool Px4ActuatorBridge::Px4IsOffboard() const {
  return status_received_ &&
         latest_status_.nav_state ==
             px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
}

bool Px4ActuatorBridge::Px4IsHealthy() const {
  return status_received_ && !latest_status_.failsafe;
}

uint64_t Px4ActuatorBridge::TimestampMicros() const {
  return static_cast<uint64_t>(now().nanoseconds() / 1000);
}

std::string Px4ActuatorBridge::Px4Topic(const std::string &suffix) const {
  return px4_topic_prefix_ + suffix;
}

const char *Px4ActuatorBridge::StateName(State state) {
  switch (state) {
  case State::kIdle:
    return "IDLE";
  case State::kPrestream:
    return "PRESTREAM";
  case State::kWaitingForOffboard:
    return "WAITING_FOR_OFFBOARD";
  case State::kWaitingForArmed:
    return "WAITING_FOR_ARMED";
  case State::kActive:
    return "ACTIVE";
  case State::kDisarming:
    return "DISARMING";
  }
  return "UNKNOWN";
}

const char *Px4ActuatorBridge::AckResultName(uint8_t result) {
  using Ack = px4_msgs::msg::VehicleCommandAck;
  switch (result) {
  case Ack::VEHICLE_CMD_RESULT_ACCEPTED:
    return "ACCEPTED";
  case Ack::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
    return "TEMPORARILY_REJECTED";
  case Ack::VEHICLE_CMD_RESULT_DENIED:
    return "DENIED";
  case Ack::VEHICLE_CMD_RESULT_UNSUPPORTED:
    return "UNSUPPORTED";
  case Ack::VEHICLE_CMD_RESULT_FAILED:
    return "FAILED";
  case Ack::VEHICLE_CMD_RESULT_IN_PROGRESS:
    return "IN_PROGRESS";
  case Ack::VEHICLE_CMD_RESULT_CANCELLED:
    return "CANCELLED";
  default:
    return "UNKNOWN";
  }
}

} // namespace ustrov_core
