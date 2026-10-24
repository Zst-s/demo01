#include "rm_behavior_tree/plugins/action/pub_current_location.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include <sstream>

namespace rm_behavior_tree
{

PubCurrentLocationAction::PubCurrentLocationAction(const std::string& name, const BT::NodeConfig& config,
                                                   const BT::RosNodeParams& params)
  : BT::SyncActionNode(name, config), node_(params.nh)
{
  if (!node_)
  {
    throw BT::RuntimeError("PubCurrentLocationAction requires a valid ROS node");
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus PubCurrentLocationAction::tick()
{
  try
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try
    {
      // Use a timeout for transform lookup
      transformStamped = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero, tf2::durationFromSec(0.5));
    }
    catch (tf2::TransformException& ex)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not transform base_link to map: %s", ex.what());
      return BT::NodeStatus::FAILURE;
    }

    // 获取位置信息
    double x = transformStamped.transform.translation.x;
    double y = transformStamped.transform.translation.y;

    // 按指定格式构建输出字符串: "x;y;0;0;0;0;1"
    std::stringstream location_str;
    location_str << x << ";" << y << ";0;0;0;0;1";

    std::string result = location_str.str();
    setOutput("current_location", result);

    // RCLCPP_INFO(node_->get_logger(), "机器人当前位置: %s", result.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node_->get_logger(), "Unexpected error: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PubCurrentLocationAction, "PubCurrentLocationAction");