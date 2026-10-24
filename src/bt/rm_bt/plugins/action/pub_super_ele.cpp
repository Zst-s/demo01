#include "rm_behavior_tree/plugins/action/pub_super_ele.hpp"

namespace pb2025_sentry_behavior
{

PubSuperEleAction::PubSuperEleAction(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
  : RosTopicPubNode<std_msgs::msg::UInt8>(name, conf, params)
{
}

bool PubSuperEleAction::setMessage(std_msgs::msg::UInt8& msg)
{
  int value = 0;
  getInput<int>("super_ele", value);
  msg.data = static_cast<uint8_t>(value);

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubSuperEleAction, "pub_super_ele");
