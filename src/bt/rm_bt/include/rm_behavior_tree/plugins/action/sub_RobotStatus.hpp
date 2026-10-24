#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOTSTATUS_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOTSTATUS_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace rm_behavior_tree
{
/**
 * @brief 该 action 订阅我方与敌方基地血量
 * @param[in] topic_name 输入端 订阅我方与敌方基地血量的话题名
 * @param[in] bothcampHp 输出端 输出我方与敌方基地血量
 */
class SubRobotStatusAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>
{
public:
  SubRobotStatusAction(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("topic_name"),
             BT::OutputPort<std::uint16_t>("current_hp"),
             BT::OutputPort<std::uint16_t>("outpost_hp"),
             BT::OutputPort<std::uint16_t>("en_outpost_hp"),
             BT::OutputPort<std::uint16_t>("bullets_fired"),
             BT::OutputPort<std::uint16_t>("remain_gold"),
             BT::OutputPort<std::uint16_t>("fort_fired"),
             BT::OutputPort<std::uint8_t>("current_stance"),
             BT::OutputPort<std::uint8_t>("ifoutpost"),
             BT::OutputPort<std::uint8_t>("in_defense_re_time"),
             BT::OutputPort<std::uint16_t>("resur_gold") };
  }
  BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus>& last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_ROBOTSTATUS_HPP_