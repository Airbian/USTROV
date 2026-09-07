#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ustrov_core/px4_actuator_bridge.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // spin 持续处理订阅、服务和 50 Hz 输出定时器，直到收到退出信号。
  rclcpp::spin(std::make_shared<ustrov_core::Px4ActuatorBridge>());
  rclcpp::shutdown();
  return 0;
}
