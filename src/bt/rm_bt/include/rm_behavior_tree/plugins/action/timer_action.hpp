#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__TIMER_ACTION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__TIMER_ACTION_HPP_

#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace rm_behavior_tree
{

class TimerAction : public BT::SyncActionNode
{
public:
  TimerAction(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<double>("elapsed_time", "从启动到现在经过的时间（秒）")
    };
  }

private:
  std::chrono::steady_clock::time_point start_time_;
  bool initialized_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__TIMER_ACTION_HPP_
