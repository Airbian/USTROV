#pragma once

#include <gz/ustrov_gz_plugins/range_measurement_array.pb.h>

#include <gz/transport/Node.hh>
#include <ustrov_control_msgs/msg/range_measurement_array.hpp>
#include <rclcpp/node_interfaces/node_topics.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ustrov_gz_plugins {
namespace range_sensor_bridge {
class RangeSensor : public rclcpp::Node {
 public:
  explicit RangeSensor(rclcpp::NodeOptions const &_options);

 private:
  void DeclareParams();
  void OnRanges(const gz::ustrov_gz_plugins::RangeMeasurementArray &_msg);
  rclcpp::Publisher<ustrov_control_msgs::msg::RangeMeasurementArray>::SharedPtr
      ranges_pub_;
  rclcpp::node_interfaces::NodeTopics *node_topics;
  std::shared_ptr<gz::transport::Node> gz_node_ =
      std::make_shared<gz::transport::Node>();
};
}  // namespace range_sensor_bridge
}  // namespace ustrov_gz_plugins
