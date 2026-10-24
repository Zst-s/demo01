#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IF_QIFU_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IF_QIFU_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace pb2025_sentry_behavior
{
class PubIfQifuAction : public BT::RosTopicPubNode<std_msgs::msg::UInt8>
{
public:
  PubIfQifuAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params);

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<int>("if_qifu", "0 或 1，表示是否激活起伏路段") });
  }

  bool setMessage(std_msgs::msg::UInt8& if_qifu) override;

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

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_IF_QIFU_HPP_