#include "rm_behavior_tree/plugins/action/pub_if_qifu.hpp"

namespace pb2025_sentry_behavior
{

PubIfQifuAction::PubIfQifuAction(const std::string& name, const BT::NodeConfig& conf,
                                     const BT::RosNodeParams& params)
  : RosTopicPubNode<std_msgs::msg::UInt8>(name, conf, params)
{
}

bool PubIfQifuAction::setMessage(std_msgs::msg::UInt8& msg)
{
  int value = 0;
  getInput<int>("if_qifu", value);
  msg.data = static_cast<uint8_t>(value);

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PubIfQifuAction, "pub_if_qifu");
