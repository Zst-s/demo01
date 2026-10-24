#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class GoalPoseBridge : public rclcpp::Node
{
public:
  GoalPoseBridge()
  : rclcpp::Node("goal_pose_bridge")
  {
    input_topic_  = this->declare_parameter<std::string>("input_topic", "goal_pose");
    action_name_  = this->declare_parameter<std::string>("action_name", "navigate_to_pose");
    cancel_prev_  = this->declare_parameter<bool>("cancel_previous", true);

    // 独立回调组，避免回调竞争
    auto cbg = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = cbg;

    sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&GoalPoseBridge::onPose, this, std::placeholders::_1),
      opts);

    client_ = rclcpp_action::create_client<NavigateToPose>(this, action_name_);
    RCLCPP_INFO(this->get_logger(),
      "Bridge: [%s] -> action [%s]", input_topic_.c_str(), action_name_.c_str());
  }

private:
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!client_->wait_for_action_server(std::chrono::milliseconds(200))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Nav2 action server not ready");
      return;
    }

    // 原来: if (cancel_prev_ && goal_handle_ && goal_handle_->is_active()) {
    if (cancel_prev_ && goal_handle_ && goal_active_) {
      client_->async_cancel_goal(goal_handle_);
      goal_active_ = false;
    }

    NavigateToPose::Goal goal;
    goal.pose = *msg;
    if (goal.pose.header.frame_id.empty()) {
      goal.pose.header.frame_id = "map";
    }

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

    // 接收是否被接受
    opts.goal_response_callback =
      [this](typename rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr gh)
      {
        if (!gh) {
          RCLCPP_WARN(this->get_logger(), "Goal rejected by server");
          goal_active_ = false;
          return;
        }
        goal_handle_ = gh;
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal accepted");
      };

    opts.feedback_callback =
      [this](auto /*gh*/, const std::shared_ptr<const NavigateToPose::Feedback> fb)
      {
        RCLCPP_DEBUG(this->get_logger(), "Nav2 feedback: distance_remaining=%.2f",
                     fb ? fb->distance_remaining : 0.0);
      };

    opts.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult & result)
      {
        goal_active_ = false;
        using rclcpp_action::ResultCode;
        switch (result.code) {
          case ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Nav2 SUCCEEDED");
            break;
          case ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Nav2 ABORTED");
            break;
          case ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Nav2 CANCELED");
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Nav2 result UNKNOWN");
            break;
        }
      };

    // 异步发送，不阻塞订阅回调线程
    client_->async_send_goal(goal, opts);
  }

  std::string input_topic_;
  std::string action_name_;
  bool cancel_prev_{true};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr goal_handle_;
  std::atomic_bool goal_active_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseBridge>());
  rclcpp::shutdown();
  return 0;
}