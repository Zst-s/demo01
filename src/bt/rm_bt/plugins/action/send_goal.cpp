#include "rm_behavior_tree/plugins/action/send_goal.hpp"
#include "rm_behavior_tree/bt_conversions.hpp"

namespace rm_behavior_tree
{
    SendGoalAction::SendGoalAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosActionNode<nav2_msgs::action::NavigateToPose>(name, config, params)
        {
        //     int tries = 0;
        // while (!action_client_->wait_for_action_server(std::chrono::seconds(1)))
        // {
        //     if(++tries >= 20)
        //     {
        //         RCLCPP_ERROR(node_->get_logger(),
        //             "[SendGoalAction] navigate_to_pose 动作服务器 20 秒内未就绪");
        //         break;
        //     }
        //     RCLCPP_WARN(node_->get_logger(),
        //         "[SendGoalAction] 等待导航动作服务器中 (%d秒)...", tries);
        // }
        }
    
    bool SendGoalAction::setGoal(nav2_msgs::action::NavigateToPose::Goal & goal)
    {
        auto res = getInput<geometry_msgs::msg::PoseStamped>("goal_pose");  //BT::Expected<geometry_msgs::msg::PoseStamped>
        if (!res) {
            throw BT::RuntimeError("error reading port [goal_pose]:", res.error());
        }
        goal.pose = res.value();
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = rclcpp::Clock().now();

        std::cout << "Goal_pose: [ "
        << std::fixed << std::setprecision(1)
        << goal.pose.pose.position.x << ", "
        << goal.pose.pose.position.y << ", "
        << goal.pose.pose.position.z << ", "
        << goal.pose.pose.orientation.x << ", "
        << goal.pose.pose.orientation.y << ", "
        << goal.pose.pose.orientation.z << ", "
        << goal.pose.pose.orientation.w << " ]\n";

        return true;
    }

    void SendGoalAction::onHalt()
    {
        // 调用基类的 onHalt() 以确保基本清理逻辑执行
        RosActionNode<nav2_msgs::action::NavigateToPose>::onHalt();

        // 检查 goal_handle_ 是否有效
        if(goal_handle_)
        {
            // 获取当前 goal 状态
            const auto status = goal_handle_->get_status();
            RCLCPP_INFO(
                node_->get_logger(),
                "[SendGoalAction] onHalt: Cancelling goal current status: %d", status
            );

            if(status ==1 || status == 2)
            //GOAL_STATUS_ACCEPTED (1): 目标已被接受。   GOAL_STATUS_EXECUTING (2): 目标正在执行。
            {
                cancelGoal();       //调用基类取消方法
            }
            else{
                RCLCPP_WARN(
                    node_->get_logger(), 
                    "[SendGoalAction] Goal already in terminal state, not canceling"
                );
            }

            goal_handle_.reset();
        }

        RCLCPP_INFO(node_->get_logger(), "SendGoalAction has been halted.");
    }

    BT::NodeStatus SendGoalAction::onResultReceived(const WrappedResult & wr)
    {
        switch (wr.code)
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "success!!!success");
                return BT::NodeStatus::SUCCESS;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "[SendGoalAction] Goal Aborted");
                return BT::NodeStatus::FAILURE;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(node_->get_logger(), "[SendGoalAction] Goal Canceled");
                return BT::NodeStatus::FAILURE;
            default:
                RCLCPP_ERROR(node_->get_logger(), "[SendGoalAction] Unknown result code");
                return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus SendGoalAction::onFeedback(
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback>)
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus SendGoalAction::onFailure(BT::ActionNodeErrorCode error)
    {
        RCLCPP_ERROR(
            node_->get_logger(),
            "[SendGoalAction] onFailure: Action failed with error code: %d", error
        );
        return BT::NodeStatus::FAILURE;
    }
};

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SendGoalAction, "SendGoal");
// BT_REGISTER_ROS_NODE(rm_behavior_tree::SendGoalAction, "SendGoal");
// BT_REGISTER_NODES(factory)
// {
//     BT::RosNodeParams p;
//     p.nh = rclcpp::Node::make_shared("send_goal_bt_node");
//     p.wait_for_server_timeout = std::chrono::seconds(20);   // 等待服务器 20 秒
//     p.server_timeout          = std::chrono::seconds(120);  // 结果超时
//     p.default_port_value      = "navigate_to_pose";
//     factory.registerNodeType<rm_behavior_tree::SendGoalAction>("SendGoal", p);
// }