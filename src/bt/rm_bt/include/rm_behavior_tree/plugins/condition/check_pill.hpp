#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_PILL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_PILL_HPP_

#include <behaviortree_cpp/condition_node.h>
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>


namespace rm_behavior_tree
{
    class CheckPillCondition : public BT::SimpleConditionNode
    {
        public:
        CheckPillCondition(const std::string & name, const BT::NodeConfig & config);
        BT::NodeStatus CheckPill();

        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<std::uint16_t>("bullets_fired"),
                BT::InputPort<int>("fired_threshold")
            };
        }
    };
}

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_PILL_HPP_
