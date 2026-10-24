#ifndef RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_REMAIN_TIME_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_REMAIN_TIME_HPP_

#include <behaviortree_cpp/condition_node.h>
#include <string>

namespace rm_behavior_tree
{

class CheckRemainTimeCondition : public BT::SimpleConditionNode
{
public:
  CheckRemainTimeCondition(const std::string& name, const BT::NodeConfig& config);

  BT::NodeStatus CheckRemainTime();

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<uint16_t>("remain_time", "当前剩余时间（秒）"),
      BT::InputPort<uint16_t>("target_time", "目标时间（秒）"),
      BT::InputPort<std::string>("operator", "less_than", "比较运算符：less_than, greater_than, equal")
    };
  }
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__CONDITION__CHECK_REMAIN_TIME_HPP_
