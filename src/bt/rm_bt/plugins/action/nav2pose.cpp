#include "rm_behavior_tree/plugins/action/nav2pose.hpp"

using namespace BT;

namespace rm_behavior_tree
{

std::atomic<int> Nav2Pose::instance_counter_{ 0 };

Nav2Pose::Nav2Pose(const std::string& name, const NodeConfig& config, std::shared_ptr<rclcpp::Node> parent_node)
  : StatefulActionNode(name, config), node_(parent_node)
{
  instance_id_ = instance_counter_.fetch_add(1);

  callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose", callback_group_);

  node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 10000);
  RCLCPP_INFO(node_->get_logger(), "Nav2Pose[%d] created", instance_id_);
}

NodeStatus Nav2Pose::onStart()
{
  // 增加序列号，使旧回调无效
  uint64_t seq = current_goal_seq_.fetch_add(1) + 1;

  // 重置状态
  goal_received_.store(false);
  goal_accepted_.store(false);
  result_received_.store(false);
  result_code_.store(0);

  if (!action_client_->wait_for_action_server(std::chrono::milliseconds(send_goal_timeout_)))
  {
    RCLCPP_ERROR(node_->get_logger(), "Nav2Pose[%d] action server unavailable", instance_id_);
    return NodeStatus::FAILURE;
  }

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal)
  {
    RCLCPP_ERROR(node_->get_logger(), "Nav2Pose[%d] goal not set", instance_id_);
    return NodeStatus::FAILURE;
  }

  nav2_msgs::action::NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal.value();
  if (nav_goal.pose.header.frame_id.empty())
  {
    nav_goal.pose.header.frame_id = "map";
  }
  nav_goal.pose.header.stamp = node_->now();

  RCLCPP_INFO(node_->get_logger(), "Nav2Pose[%d] navigating to [%.2f, %.2f]", instance_id_,
              nav_goal.pose.pose.position.x, nav_goal.pose.pose.position.y);

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.goal_response_callback = [this, seq](const NavigateToPoseGoalHandle::SharedPtr& handle) {
    if (current_goal_seq_.load() != seq)
      return;
    goal_received_.store(true);
    goal_accepted_.store(handle != nullptr);
  };

  send_goal_options.result_callback = [this, seq](const NavigateToPoseGoalHandle::WrappedResult& result) {
    if (current_goal_seq_.load() != seq)
      return;
    result_code_.store(static_cast<int>(result.code));
    result_received_.store(true);
  };

  action_client_->async_send_goal(nav_goal, send_goal_options);

  return NodeStatus::RUNNING;
}

NodeStatus Nav2Pose::onRunning()
{
  // 等待 goal response
  if (!goal_received_.load())
  {
    return NodeStatus::RUNNING;
  }

  // 检查 goal 是否被接受
  if (!goal_accepted_.load())
  {
    RCLCPP_ERROR(node_->get_logger(), "Nav2Pose[%d] goal rejected", instance_id_);
    return NodeStatus::FAILURE;
  }

  // 检查结果
  if (result_received_.load())
  {
    int code = result_code_.load();

    if (code == static_cast<int>(rclcpp_action::ResultCode::SUCCEEDED))
    {
      RCLCPP_INFO(node_->get_logger(), "Nav2Pose[%d] SUCCEEDED", instance_id_);
      return NodeStatus::SUCCESS;
    }
    else if (code == static_cast<int>(rclcpp_action::ResultCode::ABORTED))
    {
      RCLCPP_WARN(node_->get_logger(), "Nav2Pose[%d] ABORTED", instance_id_);
      return NodeStatus::FAILURE;
    }
    else if (code == static_cast<int>(rclcpp_action::ResultCode::CANCELED))
    {
      RCLCPP_WARN(node_->get_logger(), "Nav2Pose[%d] CANCELED", instance_id_);
      return NodeStatus::FAILURE;
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "Nav2Pose[%d] UNKNOWN result: %d", instance_id_, code);
      return NodeStatus::FAILURE;
    }
  }

  return NodeStatus::RUNNING;
}

void Nav2Pose::onHalted()
{
  RCLCPP_INFO(node_->get_logger(), "Nav2Pose[%d] halted", instance_id_);
  current_goal_seq_.fetch_add(1);
  goal_received_.store(false);
  goal_accepted_.store(false);
  result_received_.store(false);
}

PortsList Nav2Pose::providedPorts()
{
  return { InputPort<geometry_msgs::msg::PoseStamped>("goal", "Navigation goal") };
}

}  // namespace rm_behavior_tree
