#include "rm_behavior_tree/plugins/action/sub_current_stance.hpp"
#include <std_msgs/msg/u_int8.hpp>

namespace rm_behavior_tree
{
    SubCurrentStanceAction::SubCurrentStanceAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<rm_decision_interfaces::msg::BothCampHp>(name, config, params)
    {
    }

    BT::NodeStatus SubCurrentStanceAction::onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::BothCampHp> & last_msg)
    {
        if (last_msg)
        {
            setOutput<std::uint8_t>("current_stance", static_cast<std::uint8_t>(last_msg->current_stance));
        }else{
            setOutput<std::uint8_t>("current_stance", 3);
        }
        return BT::NodeStatus::SUCCESS;
    }
}  // namespace rm_behavior_tree

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubCurrentStanceAction, "SubCurrentStance");
// BT_REGISTER_ROS_NODE(rm_behavior_tree::SubCurrentStanceAction, "SubCurrentStance");
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<rm_behavior_tree::SubCurrentStanceAction>("SubCurrentStance");
// }