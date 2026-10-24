#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_ELAPSED_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_ELAPSED_TIME_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace rm_behavior_tree
{

class CheckElapsedTimeCondition : public BT::SimpleConditionNode
{
public:
  CheckElapsedTimeCondition(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus CheckElapsedTime();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("elapsed_time", "从启动开始经过的时间（秒）"),
      BT::InputPort<double>("target_time", "目标时间（秒）"),
      BT::InputPort<std::string>("operator", "greater_than", "比较运算符：less_than, greater_than, equal")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_ELAPSED_TIME_HPP_
