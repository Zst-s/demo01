#ifndef RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_
#define RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

namespace rm_behavior_tree
{
    class SendGoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
    {
        public:
        SendGoalAction(
            const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params
        );

        static BT::PortsList providedPorts()
        {
            return{
                BT::InputPort<geometry_msgs::msg::PoseStamped>("goal_pose"),
                BT::InputPort<std::string>("action_name")
            };
        }

        bool setGoal(Goal & goal) override;     //IDLE->RUNNING 调用一次

        void onHalt() override;     //上层打断或中断节点
        BT::NodeStatus onResultReceived(const WrappedResult & wr) override;     //结果回调
        BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override; //Action执行中反馈回调，return RUNNING  
        BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;       //失败回调    
    };
}

#endif  // RM_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_GOAL_HPP_