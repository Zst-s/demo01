#include "rm_behavior_tree/plugins/condition/check_elapsed_time.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace rm_behavior_tree
{

CheckElapsedTimeCondition::CheckElapsedTimeCondition(const std::string& name, const BT::NodeConfig& config)
  : BT::SimpleConditionNode(name, std::bind(&CheckElapsedTimeCondition::CheckElapsedTime, this), config)
{
}

BT::NodeStatus CheckElapsedTimeCondition::CheckElapsedTime()
{
  auto elapsed_time = getInput<double>("elapsed_time");
  auto target_time = getInput<double>("target_time");
  auto op = getInput<std::string>("operator");

  if (!elapsed_time || !target_time)
  {
    std::cout << "[CheckElapsedTime] 缺少输入参数: elapsed_time 或 target_time" << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  std::string operation = op.value_or("greater_than");

  bool condition_met = false;

  if (operation == "less_than")
  {
    condition_met = (elapsed_time.value() < target_time.value());
    std::cout << "[CheckElapsedTime] 已运行(" << elapsed_time.value()
              << "秒) < 目标时间(" << target_time.value() << "秒) ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else if (operation == "greater_than")
  {
    condition_met = (elapsed_time.value() > target_time.value());
    std::cout << "[CheckElapsedTime] 已运行(" << elapsed_time.value()
              << "秒) > 目标时间(" << target_time.value() << "秒) ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else if (operation == "equal")
  {
    condition_met = (elapsed_time.value() == target_time.value());
    std::cout << "[CheckElapsedTime] 已运行(" << elapsed_time.value()
              << "秒) == 目标时间(" << target_time.value() << "秒) ? "
              << (condition_met ? "是" : "否") << std::endl;
  }
  else
  {
    std::cout << "[CheckElapsedTime] 未知运算符: " << operation << std::endl;
    return BT::NodeStatus::FAILURE;
  }

  return condition_met ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::CheckElapsedTimeCondition>("CheckElapsedTime");
}
