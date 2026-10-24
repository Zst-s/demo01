#include "rm_behavior_tree/plugins/condition/check_hp.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace rm_behavior_tree
{
CheckHpCondition::CheckHpCondition(const std::string& name, const BT::NodeConfig& config)
  : BT::SimpleConditionNode(name, std::bind(&CheckHpCondition::CheckHp, this), config)
{
}

BT::NodeStatus CheckHpCondition::CheckHp()
{
  auto hp = getInput<std::uint16_t>("hp");
  auto blood_threshold = getInput<int>("blood_threshold");
  auto full_blood = getInput<int>("full_blood");

  if (!hp || !blood_threshold)
  {
    std::cout << "[CheckHp] missing input: hp or blood_threshold" << std::endl;
    return BT::NodeStatus::FAILURE;
  }
  if (hp.value() <= blood_threshold.value())
  {
    std::cout << "[CheckHp] hp(" << hp.value() << ") <= blood_threshold(" << blood_threshold.value()
              << "), condition SUCCESS" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }
}
}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CheckHpCondition>("CheckHp");
}