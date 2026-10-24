#include "rm_behavior_tree/plugins/condition/check_pill.hpp"
#include <behaviortree_cpp/bt_factory.h>


namespace rm_behavior_tree
{
    CheckPillCondition::CheckPillCondition(
        const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&CheckPillCondition::CheckPill, this), config)
    {
    }

    BT::NodeStatus CheckPillCondition::CheckPill()
    {
        auto bullets_fired = getInput<std::uint16_t>("bullets_fired");
        auto fired_threshold = getInput<int>("fired_threshold");

        if(bullets_fired.value() <= fired_threshold.value())
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::CheckPillCondition>("CheckPill");
}