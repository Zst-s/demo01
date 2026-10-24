#ifndef RM_BEHAVIOR_TREE_PLUGINS_CONTROL_RANDOMSELECTOR_HPP
#define RM_BEHAVIOR_TREE_PLUGINS_CONTROL_RANDOMSELECTOR_HPP

// Additional includes for RandomSelector
#include <behaviortree_cpp/control_node.h>
#include <limits>
#include <random>
#include <string>
#include <vector>

namespace rm_behavior_tree
{
class RandomSelector : public BT::ControlNode
{
public:
  RandomSelector(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override;
  void halt() override;

private:
  void prepareOrder();

  std::vector<std::size_t> order_;
  std::mt19937 rng_{ std::random_device{}() };
  std::size_t current_child_index_ = 0;
  std::size_t active_child_pos_ = std::numeric_limits<std::size_t>::max();
  bool need_reshuffle_ = false;
};
}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE_PLUGINS_CONTROL_RANDOMSELECTOR_HPP
