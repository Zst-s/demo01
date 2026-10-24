#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CURRENT_LOCATION_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CURRENT_LOCATION_HPP_

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include <vector>
#include <cmath>

namespace rm_behavior_tree
{

class PubCurrentLocationAction : public BT::SyncActionNode
{
public:
  // 使用外部传入的 ROS 节点
  PubCurrentLocationAction(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

  /**
   * @brief 行为树节点的 tick 方法
   * @return BT::NodeStatus
   */
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::string>("current_location")  // 仅输出位置信息
    };
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace rm_behavior_tree

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__PUB_CURRENT_LOCATION_HPP_
