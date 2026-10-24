#include "rm_behavior_tree/plugins/action/timer_action.hpp"
#include <behaviortree_cpp/bt_factory.h>
#include <iostream>

namespace rm_behavior_tree
{

TimerAction::TimerAction(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config), initialized_(false)
{
  std::cout << "[TimerAction] 创建定时器节点" << std::endl;
}

BT::NodeStatus TimerAction::tick()
{
  // 第一次调用时初始化计时器
  if (!initialized_)
  {
    start_time_ = std::chrono::steady_clock::now();
    initialized_ = true;
    std::cout << "[TimerAction] 计时器启动" << std::endl;
  }

  // 计算经过的时间（秒）
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
  double elapsed_seconds = elapsed.count();

  // 输出到端口
  setOutput("elapsed_time", elapsed_seconds);

  std::cout << "[TimerAction] 已运行: " << elapsed_seconds << " 秒" << std::endl;

  return BT::NodeStatus::SUCCESS;
}

}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::TimerAction>("Timer");
}
