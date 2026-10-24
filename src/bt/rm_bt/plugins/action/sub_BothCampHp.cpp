#include "rm_behavior_tree/plugins/action/sub_BothCampHp.hpp"
#include <std_msgs/msg/u_int16.hpp>

namespace rm_behavior_tree
{
    SubBothCampHpAction::SubBothCampHpAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<rm_decision_interfaces::msg::BothCampHp>(name, config, params)
    {
    }

    BT::NodeStatus SubBothCampHpAction::onTick(
        const std::shared_ptr<rm_decision_interfaces::msg::BothCampHp> & last_msg
    )
    {
        if(last_msg)
        {
            RCLCPP_DEBUG(
                logger(),
                "我方哨兵血量: [%d]\n",
                last_msg->own_sentry_hp
            );
            RCLCPP_DEBUG(logger(), "-----------------------------------------------");
            setOutput("both_camp_hp", *last_msg);
            // 显式指定类型，避免黑板条目在不同 tick 中被写入不同类型
            setOutput<std::uint16_t>("own_sentry_hp", static_cast<std::uint16_t>(last_msg->own_sentry_hp));
        }else{
            // static rm_decision_interfaces::msg::BothCampHp default_msg;
            // default_msg.own_sentry_hp = 280;
            // setOutput("both_camp_hp", default_msg);

            // setOutput<std::uint16_t>("own_sentry_hp", static_cast<std::uint16_t>(280));
            // return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::SUCCESS;
    }
}

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubBothCampHpAction, "SubBothCampHp");
// BT_REGISTER_ROS_NODE(rm_behavior_tree::SubBothCampHpAction, "SubBothCampHp");
// BT_REGISTER_NODES(factory)
// {
//     factory.registerNodeType<rm_behavior_tree::SubBothCampHpAction>("SubBothCampHp");
// }