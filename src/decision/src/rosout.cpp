#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <chrono>
#include <cstdlib>

using namespace std;
using std::placeholders::_1;

class RosoutMonitor : public rclcpp::Node
{
public:
    RosoutMonitor() : Node("rosout_node"), start_time_(std::chrono::steady_clock::now())
    {
        // 发布话题，当 bond_timer_count == 2 时发布 1
        nav_ready_pub = this->create_publisher<std_msgs::msg::UInt8>("/nav_ready", 10);

        // 接收启动期间已发布的生命周期日志，避免错过较早启动的 map_server。
        auto qos = rclcpp::QoS(100);
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
        
        rosout_sub = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", qos, std::bind(&RosoutMonitor::rosout_callback, this, _1));

        // 创建定时器，每秒检查一次超时
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&RosoutMonitor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "rosout_monitor_node 已启动，等待检测 'Creating bond timer'...");
    }

private:
    // rosout 话题的回调函数，监控终端输出
    void rosout_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        if (msg->msg.find("Creating bond timer...") == std::string::npos)
        {
            return;
        }

        if (msg->name == "lifecycle_manager_localization")
        {
            localization_ready_ = true;
        }
        else if (msg->name == "lifecycle_manager_navigation")
        {
            navigation_ready_ = true;
        }
        else
        {
            return;
        }

        const int bond_timer_count = static_cast<int>(localization_ready_) +
            static_cast<int>(navigation_ready_);
        RCLCPP_INFO(this->get_logger(), "检测到 %s 已激活，bond_timer_count = %d",
                    msg->name.c_str(), bond_timer_count);
        if (!nav_ready_ && localization_ready_ && navigation_ready_)
        {
            nav_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "导航就绪！开始周期发布 /nav_ready = 1");
        }
    }

    // 定时器回调函数，检查超时
    void timer_callback()
    {
        // 导航就绪后周期发布 /nav_ready = 1，保证晚于本节点启动的串口节点也能收到
        if (nav_ready_)
        {
            std_msgs::msg::UInt8 ready_msg;
            ready_msg.data = 1;
            nav_ready_pub->publish(ready_msg);
            return;
        }

        auto elapsed = std::chrono::steady_clock::now() - start_time_;
        double elapsed_sec = std::chrono::duration<double>(elapsed).count();

        const int bond_timer_count = static_cast<int>(localization_ready_) +
            static_cast<int>(navigation_ready_);
        RCLCPP_INFO(this->get_logger(), "已等待 %.0f 秒，bond_timer_count = %d", elapsed_sec, bond_timer_count);

        if (elapsed_sec >= 25.0 && !timeout_reported_)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "25秒仍未就绪：定位=%d，导航=%d；请检查 map_server 和生命周期管理器日志",
                         localization_ready_, navigation_ready_);
            timeout_reported_ = true;
        }
    }

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr nav_ready_pub;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point start_time_;
    bool localization_ready_ = false;
    bool navigation_ready_ = false;
    bool nav_ready_ = false;
    bool timeout_reported_ = false;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosoutMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
