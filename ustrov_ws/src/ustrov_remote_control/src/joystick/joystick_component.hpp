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

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <ustrov_control_msgs/msg/actuator_setpoint.hpp>

namespace ustrov_remote_control {
namespace joystick {

namespace axes {
static constexpr std::size_t kLeftStickLeftRight = 0;
static constexpr std::size_t kLeftStickUpDown = 1;
static constexpr std::size_t kLT = 2;
static constexpr std::size_t kRightStickLeftRight = 3;
static constexpr std::size_t kRightStickUpDown = 4;
static constexpr std::size_t kRT = 5;
static constexpr std::size_t kCrossLeftRight = 6;
static constexpr std::size_t kCrossUpDown = 7;
static constexpr std::size_t kNumAxes = 8;
}  // namespace axes

class JoyStick : public rclcpp::Node {
 public:
  explicit JoyStick(const rclcpp::NodeOptions &_options);

  struct Params {
    struct Gains {
      struct Torque {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } torque;
      struct Thrust {
        double x{1.0};
        double y{1.0};
        double z{1.0};
      } thrust;
    } gains;
    struct Mapping {
      // true：按 Hippo 四推进器构型输出 Fx、Mx、My、Mz。
      // false：保持上游手柄的 Fx、Fy、Fz、Mz 映射。
      bool use_4dof{false};
    } mapping;
  };

 private:
  void DeclareParams();
  rcl_interfaces::msg::SetParametersResult OnParams(
      const std::vector<rclcpp::Parameter> _parameters);
  void InitPublishers();
  void InitSubscribers();

  std::array<double, 3> ComputeThrust(
      const std::vector<float> &_axes,
      const std::vector<int32_t> &_buttons);
  std::array<double, 3> ComputeTorque(
      const std::vector<float> &_axes,
      const std::vector<int32_t> &_buttons);
  void PublishThrust(const std::array<double, 3> &_thrust);
  void PublishTorque(const std::array<double, 3> &_torque);
  void OnJoy(const sensor_msgs::msg::Joy::SharedPtr _msg);

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
      params_cb_handle_;
  rclcpp::Publisher<ustrov_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      thrust_pub_;
  rclcpp::Publisher<ustrov_control_msgs::msg::ActuatorSetpoint>::SharedPtr
      torque_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  Params params_;
};

}  // namespace joystick
}  // namespace ustrov_remote_control
