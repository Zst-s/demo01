#include "rm_behavior_tree/plugins/action/sub_remain_time.hpp"
#include <std_msgs/msg/u_int16.hpp>

namespace rm_behavior_tree
{
    SubRemainTimeAction::SubRemainTimeAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<std_msgs::msg::UInt16>(name, config, params)
    {
    }

    BT::NodeStatus SubRemainTimeAction::onTick(
        const std::shared_ptr<std_msgs::msg::UInt16> & last_msg)
    {
        if (last_msg)
        {
            setOutput<uint16_t>("remain_time", last_msg->data);
        }else{
            setOutput<uint16_t>("remain_time", 410);
        }
        return BT::NodeStatus::SUCCESS;
    }
}  // namespace rm_behavior_tree

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubRemainTimeAction, "SubRemainTime");