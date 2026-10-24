#include "rm_behavior_tree/plugins/action/sub_RobotStatus.hpp"
#include <std_msgs/msg/u_int16.hpp>
#include <iostream>

namespace rm_behavior_tree
{
SubRobotStatusAction::SubRobotStatusAction(const std::string& name, const BT::NodeConfig& config,
                                           const BT::RosNodeParams& params)
  : BT::RosTopicSubNode<rm_decision_interfaces::msg::RobotStatus>(name, config, params)
{
}

BT::NodeStatus SubRobotStatusAction::onTick(const std::shared_ptr<rm_decision_interfaces::msg::RobotStatus>& last_msg)
{
  if (last_msg)
  {
    // 显式指定类型，避免黑板条目在不同 tick 中被写入不同类型
    const auto current_hp = static_cast<std::uint16_t>(last_msg->current_hp);
    const auto outpost_hp = static_cast<std::uint16_t>(last_msg->outpost_hp);
    const auto en_outpost_hp = static_cast<std::uint16_t>(last_msg->en_outpost_hp);
    const auto bullets_fired = static_cast<std::uint16_t>(last_msg->bullets_fired);
    const auto remain_gold = static_cast<std::uint16_t>(last_msg->remain_gold);
    const auto fort_fired = static_cast<std::uint16_t>(last_msg->fort_fired);
    const auto resur_gold = static_cast<std::uint16_t>(last_msg->resur_gold);
    const auto current_stance = static_cast<std::uint8_t>(last_msg->current_stance);
    const auto ifoutpost = static_cast<std::uint8_t>(last_msg->ifoutpost);
    const auto in_defense_re_time = static_cast<std::uint8_t>(last_msg->in_defense_re_time);

    setOutput<std::uint16_t>("current_hp", current_hp);
    setOutput<std::uint16_t>("outpost_hp", outpost_hp);
    setOutput<std::uint16_t>("en_outpost_hp", en_outpost_hp);
    setOutput<std::uint16_t>("bullets_fired", bullets_fired);
    setOutput<std::uint16_t>("remain_gold", remain_gold);
    setOutput<std::uint16_t>("fort_fired", fort_fired);
    setOutput<std::uint16_t>("resur_gold", resur_gold);
    setOutput<std::uint8_t>("current_stance", current_stance);
    setOutput<std::uint8_t>("ifoutpost", ifoutpost);
    setOutput<std::uint8_t>("in_defense_re_time", in_defense_re_time);

    // std::cout << "[SubRobotStatus] current_hp(" << current_hp
    //           << ") outpost_hp(" << outpost_hp
    //           << ") en_outpost_hp(" << en_outpost_hp
    //           << ") bullets_fired(" << bullets_fired
    //           << ") remain_gold(" << remain_gold
    //           << ") fort_fired(" << fort_fired
    //           << ") resur_gold(" << resur_gold
    //           << ") current_stance(" << static_cast<int>(current_stance)
    //           << ")" << std::endl;
  }
  else
  {
    static rm_decision_interfaces::msg::RobotStatus default_msg;
    default_msg.current_hp = 400;
    default_msg.outpost_hp = 1500;
    default_msg.en_outpost_hp = 1500;
    default_msg.bullets_fired = 300;
    default_msg.remain_gold = 1000;
    default_msg.fort_fired = 100;
    default_msg.resur_gold = 100;
    default_msg.current_stance = 3;
    default_msg.ifoutpost = 0;
    default_msg.in_defense_re_time = 0;

    setOutput("current_hp", default_msg.current_hp);
    setOutput("outpost_hp", default_msg.outpost_hp);
    setOutput("en_outpost_hp", default_msg.en_outpost_hp);
    setOutput("bullets_fired", default_msg.bullets_fired);
    setOutput("remain_gold", default_msg.remain_gold);
    setOutput("fort_fired", default_msg.fort_fired);
    setOutput("resur_gold", default_msg.resur_gold);
    setOutput("current_stance", default_msg.current_stance);
    setOutput("ifoutpost", default_msg.ifoutpost);
    setOutput("in_defense_re_time", default_msg.in_defense_re_time);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::SUCCESS;
}
}  // namespace rm_behavior_tree

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubRobotStatusAction, "SubRobotStatus");
