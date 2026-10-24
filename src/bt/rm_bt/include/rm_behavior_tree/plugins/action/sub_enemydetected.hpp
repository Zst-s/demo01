#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ENEMYDETECTED_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ENEMYDETECTED_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>
#include "rm_decision_interfaces/msg/enemy_detected.hpp"

namespace rm_behavior_tree
{
    /**
     * @brief 该 action 订阅识别敌人状态
     * @param[in] topic_name 输入端 订阅识别敌人状态的话题名
     * @param[in] bothcampHp 输出端 输出识别状态    识别到 1    未识别到 0
     */
    class SubEnemyDetectedAction : public BT::RosTopicSubNode<std_msgs::msg::UInt8>
    {
    public:
        SubEnemyDetectedAction(
            const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
        
        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<std::string>("topic_name"),
                BT::OutputPort<std::uint8_t>("enemy_detected")
            };
        }

        BT::NodeStatus onTick(
            const std::shared_ptr<std_msgs::msg::UInt8> & last_msg
        ) override;
    };
}

#endif