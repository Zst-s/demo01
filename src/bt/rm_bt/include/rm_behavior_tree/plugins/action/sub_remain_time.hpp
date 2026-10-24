#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_REMAIN_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_REMAIN_TIME_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/int16.hpp>

namespace rm_behavior_tree
{
        /**
     * @brief 该 action 订阅剩余时间
     * @param[in] topic_name 输入端 订阅剩余时间的话题名
     * @param[in] remain_time 输出端 输出剩余时间
     */
    class SubRemainTimeAction : public BT::RosTopicSubNode<std_msgs::msg::UInt16>
    {
        public:
            SubRemainTimeAction(
                const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);
            static BT::PortsList providedPorts()
            {
                return{
                    BT::InputPort<std::string>("topic_name"),
                    BT::OutputPort<std::uint16_t>("remain_time")
                };
            }
            BT::NodeStatus onTick(
                const std::shared_ptr<std_msgs::msg::UInt16> & last_msg
            ) override;
    };
}
#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_REMAIN_TIME_HPP_