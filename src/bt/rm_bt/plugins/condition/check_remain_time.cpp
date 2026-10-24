#include "rm_behavior_tree/plugins/condition/check_remain_time.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace rm_behavior_tree
{

CheckRemainTimeCondition::CheckRemainTimeCondition(const std::string& name, const BT::NodeConfig& config)
  : BT::SimpleConditionNode(name, std::bind(&CheckRemainTimeCondition::CheckRemainTime, this), config)
{
}

BT::NodeStatus CheckRemainTimeCondition::CheckRemainTime()
{
  auto remain_time = getInput<uint16_t>("remain_time");
  auto target_time = getInput<uint16_t>("target_time");
  auto op = getInput<std::string>("operator");

  if (!remain_time || !target_time)
  {
    std::cout << "[CheckRemainTime] 缺少输入参数: remain_time 或 target_time" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string operation = op.value_or("less_than");

  bool condition_met = false;

  if (operation == "less_than")
  {
    condition_met = (remain_time.value() < target_time.value());
    std::cout << "[CheckRemainTime] 剩余时间(" << remain_time.value()
              << ") < 目标时间(" << target_time.value() << ") ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else if (operation == "greater_than")
  {
    condition_met = (remain_time.value() > target_time.value());
    std::cout << "[CheckRemainTime] 剩余时间(" << remain_time.value()
              << ") > 目标时间(" << target_time.value() << ") ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else if (operation == "equal")
  {
    condition_met = (remain_time.value() == target_time.value());
    std::cout << "[CheckRemainTime] 剩余时间(" << remain_time.value()
              << ") == 目标时间(" << target_time.value() << ") ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else
  {
    std::cout << "[CheckRemainTime] 未知运算符: " << operation << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CheckRemainTimeCondition>("CheckRemainTime");
}
