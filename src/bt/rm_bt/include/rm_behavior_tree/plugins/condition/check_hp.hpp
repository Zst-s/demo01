#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_

#include <behaviortree_cpp/condition_node.h>
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int8.hpp>


namespace rm_behavior_tree
{
    class CheckHpCondition : public BT::SimpleConditionNode
    {
        public:
        CheckHpCondition(const std::string & name, const BT::NodeConfig & config);
        BT::NodeStatus CheckHp();

        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<std::uint16_t>("hp"),
                BT::InputPort<int>("blood_threshold"),
                BT::InputPort<int>("full_blood")
            };
        }
    };
}

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_HP_HPP_
