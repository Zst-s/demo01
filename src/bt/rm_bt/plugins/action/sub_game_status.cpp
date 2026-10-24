#include "rm_behavior_tree/plugins/action/sub_game_status.hpp"
#include <std_msgs/msg/u_int8.hpp>

namespace rm_behavior_tree
{
    SubGameStatusAction::SubGameStatusAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<std_msgs::msg::UInt8>(name, config, params)
    {
    }

    BT::NodeStatus SubGameStatusAction::onTick(
        const std::shared_ptr<std_msgs::msg::UInt8> & last_msg)
    {
        if (last_msg)
        {
            setOutput<uint8_t>("game_status", last_msg->data);
        }else{
            setOutput<uint8_t>("game_status", 3);
        }
        return BT::NodeStatus::SUCCESS;
    }
}  // namespace rm_behavior_tree

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");
// BT_REGISTER_ROS_NODE(rm_behavior_tree::SubGameStatusAction, "SubGameStatus");
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<rm_behavior_tree::SubGameStatusAction>("SubGameStatus");
// }