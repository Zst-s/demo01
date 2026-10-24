#include "rm_behavior_tree/plugins/action/sub_fort_if.hpp"
#include <std_msgs/msg/u_int8.hpp>

namespace rm_behavior_tree
{
    SubFortIfAction::SubFortIfAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<std_msgs::msg::UInt8>(name, config, params)
    {
    }

    BT::NodeStatus SubFortIfAction::onTick(
        const std::shared_ptr<std_msgs::msg::UInt8> & last_msg)
    {
        if (last_msg)
        {
            setOutput<uint8_t>("fort_if", last_msg->data);
        }else{
            setOutput<uint8_t>("fort_if", 0);
        }
        return BT::NodeStatus::SUCCESS;
    }
}  // namespace rm_behavior_tree

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubFortIfAction, "SubFortIf");