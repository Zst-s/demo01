#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CURRENT_STANCE_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CURRENT_STANCE_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "rm_decision_interfaces/msg/both_camp_hp.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace rm_behavior_tree
{
        /**
     * @brief 该 action 订阅当前姿态
     * @param[in] topic_name 输入端 订阅当前姿态的话题名
     * @param[in] current_stance 输出端 输出当前姿态 {0, 默认} {1, 进攻姿态} {2, 防御姿态} {3, 移动姿态}
     */
    class SubCurrentStanceAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::BothCampHp>
    {
        public:
            SubCurrentStanceAction(
                const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
            static BT::PortsList providedPorts()
            {
                return{
                    BT::InputPort<std::string>("topic_name"),
                    BT::OutputPort<std::uint8_t>("current_stance")
                };
            }
            BT::NodeStatus onTick(
                const std::shared_ptr<rm_decision_interfaces::msg::BothCampHp> & last_msg
            ) override;
    };
}
#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_CURRENT_STANCE_HPP_