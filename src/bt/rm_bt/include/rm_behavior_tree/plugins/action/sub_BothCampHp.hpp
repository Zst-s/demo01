#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BOTHCAMPHP_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BOTHCAMPHP_HPP_

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "rm_decision_interfaces/msg/both_camp_hp.hpp"
#include <std_msgs/msg/int16.hpp>

namespace rm_behavior_tree
{
/**
 * @brief 该 action 订阅我方与敌方基地血量
 * @param[in] topic_name 输入端 订阅我方与敌方基地血量的话题名
 * @param[in] bothcampHp 输出端 输出我方与敌方基地血量
 */
class SubBothCampHpAction : public BT::RosTopicSubNode<rm_decision_interfaces::msg::BothCampHp>
{
public:
  SubBothCampHpAction(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("topic_name"),
             BT::OutputPort<rm_decision_interfaces::msg::BothCampHp>("both_camp_hp"),
             BT::OutputPort<std::uint16_t>("own_sentry_hp") };
  }
  BT::NodeStatus onTick(const std::shared_ptr<rm_decision_interfaces::msg::BothCampHp>& last_msg) override;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SUB_BOTHCAMPHP_HPP_