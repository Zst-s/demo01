#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_INTENSIFY_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_INTENSIFY_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace rm_behavior_tree
{
class PubIntensifyAction : public BT::RosTopicPubNode<std_msgs::msg::UInt8>
{
public:
  PubIntensifyAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>("intensify", "0 或 1，表示是否激活强化模式") });
  }

  bool setMessage(std_msgs::msg::UInt8& intensify) override;

private:
  rclcpp::Logger logger()
  {
    return node_->get_logger();
  }
  rclcpp::Time now()
  {
    return node_->now();
  }
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_INTENSIFY_HPP_