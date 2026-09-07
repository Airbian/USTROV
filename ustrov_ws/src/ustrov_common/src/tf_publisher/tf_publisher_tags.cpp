#include "tf_publisher_tags.hpp"

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ustrov_common/param_utils.hpp>
#include <ustrov_common/yaml.hpp>

#include "ustrov_common/convert.hpp"

namespace ustrov_common {

TfPublisherTags::TfPublisherTags(const rclcpp::NodeOptions &_options)
    : Node("tf_publisher_tags", _options) {
  static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  USTROV_COMMON_DECLARE_PARAM_READONLY(tag_poses_file);
  BroadCastStatic();
}

void TfPublisherTags::BroadCastStatic() {
  YAML::Node node;
  try {
    node = YAML::LoadFile(params_.tag_poses_file);
  } catch (const YAML::BadFile &) {
    RCLCPP_FATAL(get_logger(), "Could not load tag_poses file [%s]",
                 params_.tag_poses_file.c_str());
    return;
  }
  auto tag_poses = node["tag_poses"].as<ustrov_common::yaml::TagPoses>();

  std::vector<geometry_msgs::msg::TransformStamped> transforms;
  geometry_msgs::msg::TransformStamped t;
  char buffer[255];
  for (const auto &tag_pose : tag_poses) {
    ustrov_common::convert::EigenToRos(tag_pose.position,
                                      t.transform.translation);
    ustrov_common::convert::EigenToRos(tag_pose.orientation,
                                      t.transform.rotation);
    snprintf(buffer, sizeof(buffer), "ground_truth/tag_%03d", tag_pose.id);
    t.child_frame_id = std::string{buffer};
    t.header.frame_id = tag_pose.frame_id;
    transforms.push_back(t);
  }
  if (!static_broadcaster_) {
    RCLCPP_FATAL(get_logger(), "Broadcaster not initialized");
    return;
  }
  static_broadcaster_->sendTransform(transforms);
}

}  // namespace ustrov_common

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ustrov_common::TfPublisherTags)
