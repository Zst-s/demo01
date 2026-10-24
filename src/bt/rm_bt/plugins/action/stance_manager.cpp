#include "rm_behavior_tree/plugins/action/stance_manager.hpp"
#include <behaviortree_cpp/bt_factory.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<rm_behavior_tree::UpdateStanceTime>("UpdateStanceTime");
  factory.registerNodeType<rm_behavior_tree::SwitchStance>("SwitchStance");
}