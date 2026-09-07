// Copyright (C) 2023 Thies Lennart Alff
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301,
// USA

#include "actuator_mixer_node.hpp"

using namespace ustrov_control;
using namespace ustrov_common;
using namespace ustrov_control_msgs::msg;
using namespace rcl_interfaces;
using std::placeholders::_1;

static constexpr int kTimeoutMs = 300;

namespace ustrov_control {
namespace mixer {

ActuatorMixerNode::ActuatorMixerNode(rclcpp::NodeOptions const& _options)
    : Node("actuator_command_mixer", _options) {
  RCLCPP_INFO(get_logger(), "Declaring Paramters");
  DeclareParams();
  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  std::string name;

  t_last_thrust_setpoint_ = t_last_torque_setpoint_ = now();

  name = "thruster_command";
  actuator_controls_pub_ = create_publisher<ActuatorControls>(name, qos);

  name = "thrust_setpoint";
  thrust_setpoint_sub_ =
      create_subscription<ustrov_control_msgs::msg::ActuatorSetpoint>(
          name, qos,
          std::bind(&ActuatorMixerNode::OnThrustSetpoint, this,
                    std::placeholders::_1));

  name = "torque_setpoint";
  torque_setpoint_sub_ =
      create_subscription<ustrov_control_msgs::msg::ActuatorSetpoint>(
          name, qos,
          std::bind(&ActuatorMixerNode::OnTorqueSetpoint, this,
                    std::placeholders::_1));

  watchdog_timer_ = rclcpp::create_timer(
      this, get_clock(), std::chrono::milliseconds(kTimeoutMs),
      std::bind(&ActuatorMixerNode::WatchdogTimeout, this));
  RCLCPP_INFO(get_logger(), "Initialization complete.");
}

void ActuatorMixerNode::WatchdogTimeout() {
  auto t_now = now();
  static bool timed_out_thrusts_prev{false};
  static bool timed_out_torques_prev{false};
  const bool timed_out_thrusts =
      (t_now - t_last_thrust_setpoint_).nanoseconds() * 1e-6 > kTimeoutMs;
  const bool timed_out_torques =
      (t_now - t_last_torque_setpoint_).nanoseconds() * 1e-6 > kTimeoutMs;

  bool inputs_changed = false;
  if (timed_out_thrusts) {
    ResetThrust();

    if (!timed_out_thrusts_prev) {
      RCLCPP_WARN_STREAM(
          get_logger(),
          "Thrust input messages timed out. Setting thrusts to zero.");
      inputs_changed = true;
    }
  } else if (timed_out_thrusts_prev) {
    RCLCPP_INFO(get_logger(),
                "Received new thrust input messages. Not timed out anymore.");
  }

  if (timed_out_torques) {
    ResetTorque();

    if (!timed_out_torques_prev) {
      RCLCPP_WARN_STREAM(
          get_logger(),
          "Torque input messages timed out. Setting torques to zero.");
      inputs_changed = true;
    }
  } else if (timed_out_torques_prev) {
    RCLCPP_INFO(get_logger(),
                "Received new torque input messages. Not timed out anymore.");
  }

  // 超时只发布一次更新后的零命令，之后停止发布，让 ustrov_core 的命令
  // 看门狗能够继续超时并请求 PX4 上锁。
  if (inputs_changed) {
    PublishActuatorCommand(t_now);
  }

  timed_out_thrusts_prev = timed_out_thrusts;
  timed_out_torques_prev = timed_out_torques;
}

void ActuatorMixerNode::PublishActuatorCommand(const rclcpp::Time& _now) {
  ustrov_control_msgs::msg::ActuatorControls msg;
  msg.control = mixer_.Mix(inputs_);
  msg.header.stamp = _now;
  actuator_controls_pub_->publish(msg);
}

void ActuatorMixerNode::ResetThrust() {
  inputs_[mixer::InputChannels::kThrustX] = 0.0;
  inputs_[mixer::InputChannels::kThrustY] = 0.0;
  inputs_[mixer::InputChannels::kThrustZ] = 0.0;
}
void ActuatorMixerNode::ResetTorque() {
  inputs_[mixer::InputChannels::kTorqueX] = 0.0;
  inputs_[mixer::InputChannels::kTorqueY] = 0.0;
  inputs_[mixer::InputChannels::kTorqueZ] = 0.0;
}

void ActuatorMixerNode::OnThrustSetpoint(
    const ustrov_control_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  if (!_msg->ignore_x) {
    inputs_[mixer::InputChannels::kThrustX] = _msg->x;
  }
  if (!_msg->ignore_y) {
    inputs_[mixer::InputChannels::kThrustY] = _msg->y;
  }
  if (!_msg->ignore_z) {
    inputs_[mixer::InputChannels::kThrustZ] = _msg->z;
  }
  t_last_thrust_setpoint_ = now();
  PublishActuatorCommand(t_last_thrust_setpoint_);
}

void ActuatorMixerNode::OnTorqueSetpoint(
    const ustrov_control_msgs::msg::ActuatorSetpoint::SharedPtr _msg) {
  if (!_msg->ignore_x) {
    inputs_[mixer::InputChannels::kTorqueX] = _msg->x;
  }
  if (!_msg->ignore_y) {
    inputs_[mixer::InputChannels::kTorqueY] = _msg->y;
  }
  if (!_msg->ignore_z) {
    inputs_[mixer::InputChannels::kTorqueZ] = _msg->z;
  }
  t_last_torque_setpoint_ = now();
  PublishActuatorCommand(t_last_torque_setpoint_);
}

}  // namespace mixer
}  // namespace ustrov_control
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ustrov_control::mixer::ActuatorMixerNode)
