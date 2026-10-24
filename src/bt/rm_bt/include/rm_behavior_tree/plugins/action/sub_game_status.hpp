#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace rm_behavior_tree
{
        /**
     * @brief 该 action 订阅比赛状态
     * @param[in] topic_name 输入端 订阅比赛状态的话题名
     * @param[in] game_status 输出端 输出比赛状态 {0, 未开始比赛} {1, 准备阶段} {2, 15 s 裁判系统自检阶段} {3, 5s 倒计时} {4, 比赛中} {5, 比赛结算中}
     */
    class SubGameStatusAction : public BT::RosTopicSubNode<std_msgs::msg::UInt8>
    {
        public:
            SubGameStatusAction(
                const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
            static BT::PortsList providedPorts()
            {
                return{
                    BT::InputPort<std::string>("topic_name"),
                    BT::OutputPort<std::uint8_t>("game_status")
                };
            }
            BT::NodeStatus onTick(
                const std::shared_ptr<std_msgs::msg::UInt8> & last_msg
            ) override;
    };
}
#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_GAME_STATUS_HPP_