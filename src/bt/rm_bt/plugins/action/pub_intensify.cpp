#include "rm_behavior_tree/plugins/action/pub_intensify.hpp"

namespace rm_behavior_tree
{

PubIntensifyAction::PubIntensifyAction(const std::string& name, const BT::NodeConfig& conf,
                                       const BT::RosNodeParams& params)
  : RosTopicPubNode<std_msgs::msg::UInt8>(name, conf, params)
{
}

bool PubIntensifyAction::setMessage(std_msgs::msg::UInt8& msg)
{
  int value = 0;
  getInput<int>("intensify", value);
  msg.data = static_cast<uint8_t>(value);
  if(value != 0){
    RCLCPP_INFO(node_->get_logger(), "强化intensify: %d", value);
  }
  
  return true;
}

}  // namespace rm_behavior_tree

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(rm_behavior_tree::PubIntensifyAction, "pub_intensify");
