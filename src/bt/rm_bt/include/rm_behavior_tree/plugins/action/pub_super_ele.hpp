#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SUPER_ELE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SUPER_ELE_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace pb2025_sentry_behavior
{
class PubSuperEleAction : public BT::RosTopicPubNode<std_msgs::msg::UInt8>
{
public:
  PubSuperEleAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>("super_ele", "0 或 1，表示是否激活超级电容") });
  }

  bool setMessage(std_msgs::msg::UInt8& super_ele) override;

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
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_SUPER_ELE_HPP_