#include "rm_behavior_tree/plugins/control/randomselector.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>
#include <limits>
#include <numeric>
#include <iostream>

namespace rm_behavior_tree
{
namespace
{
constexpr std::size_t kInvalidIndex = std::numeric_limits<std::size_t>::max();
}

RandomSelector::RandomSelector(const std::string& name, const BT::NodeConfig& config) : BT::ControlNode(name, config)
{
}

BT::NodeStatus RandomSelector::tick()
{
  if (children().empty())
  {
    order_.clear();
    current_child_index_ = 0;
    active_child_pos_ = kInvalidIndex;
    need_reshuffle_ = false;
    return BT::NodeStatus::FAILURE;
  }

  prepareOrder();

  // 如果有正在执行的子节点
  if (active_child_pos_ != kInvalidIndex)
  {
    const auto tracked_child = order_[active_child_pos_];
    std::cout << "[RandomSelector] Continuing child " << tracked_child << std::endl;
    const auto status = children()[tracked_child]->executeTick();
    if (status == BT::NodeStatus::RUNNING)
    {
      return BT::NodeStatus::RUNNING;
    }

    std::cout << "[RandomSelector] Child " << tracked_child << " finished with status "
              << (status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE") << ", will reshuffle next tick"
              << std::endl;
    active_child_pos_ = kInvalidIndex;
    need_reshuffle_ = true;
    return BT::NodeStatus::RUNNING;  // 等下一个 tick 再 reshuffle
  }

  // 需要 reshuffle
  if (need_reshuffle_)
  {
    // 先 halt 所有子节点
    haltChildren();
    std::shuffle(order_.begin(), order_.end(), rng_);
    current_child_index_ = 0;
    need_reshuffle_ = false;
    
    std::cout << "[RandomSelector] Reshuffled, will pick child next tick" << std::endl;
    return BT::NodeStatus::RUNNING;  // 等下一个 tick 再执行新节点
  }

  // 执行新节点
  const auto child_idx = order_[current_child_index_];
  std::cout << "[RandomSelector] Picking child " << child_idx << std::endl;
  const auto child_status = children()[child_idx]->executeTick();
  
  switch (child_status)
  {
    case BT::NodeStatus::RUNNING:
      active_child_pos_ = current_child_index_;
      return BT::NodeStatus::RUNNING;
    case BT::NodeStatus::SUCCESS:
    case BT::NodeStatus::FAILURE:
    default:
      std::cout << "[RandomSelector] Child " << child_idx << " returned immediately with "
                << (child_status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE") << std::endl;
      need_reshuffle_ = true;
      return BT::NodeStatus::RUNNING;
  }
}

void RandomSelector::halt()
{
  BT::ControlNode::halt();
  current_child_index_ = 0;
  active_child_pos_ = kInvalidIndex;
  need_reshuffle_ = false;
}

void RandomSelector::prepareOrder()
{
  const auto child_count = children().size();
  if (order_.size() != child_count)
  {
    order_.resize(child_count);
    std::iota(order_.begin(), order_.end(), 0);
    current_child_index_ = 0;
    active_child_pos_ = kInvalidIndex;
    need_reshuffle_ = true;  // 初始需要 shuffle
  }
}
}  // namespace rm_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::RandomSelector>("RandomSelector");
}
