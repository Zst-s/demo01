#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_MID_IF_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_MID_IF_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/int8.hpp>

namespace rm_behavior_tree
{
        /**
     * @brief 该 action 订阅中央占领状态
     * @param[in] topic_name 输入端 订阅中央占领状态的话题名
     * @param[in] mid_if 输出端 输出中央占领状态 {0, 未占领} {1, 己方占领} {2, 对方占领} {3, 双方占领}
     */
    class SubMidIfAction : public BT::RosTopicSubNode<std_msgs::msg::UInt8>
    {
        public:
            SubMidIfAction(
                const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
            static BT::PortsList providedPorts()
            {
                return{
                    BT::InputPort<std::string>("topic_name"),
                    BT::OutputPort<std::uint8_t>("mid_if")
                };
            }
            BT::NodeStatus onTick(
                const std::shared_ptr<std_msgs::msg::UInt8> & last_msg
            ) override;
    };
}
#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_MID_IF_HPP_