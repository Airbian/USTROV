#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ustrov_core/px4_sensor_bridge.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ustrov_core::Px4SensorBridge>());
  rclcpp::shutdown();
  return 0;
}
