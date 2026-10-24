#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include <chrono>
#include <cstdlib>
#include <string>

using namespace std::chrono_literals;

class RosoutSerialMonitor : public rclcpp::Node
{
public:
    RosoutSerialMonitor() : Node("rosout_serial_monitor")
    {
        auto qos = rclcpp::QoS(100);
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
        rosout_sub_ = this->create_subscription<rcl_interfaces::msg::Log>(
            "/rosout", qos, std::bind(&RosoutSerialMonitor::rosout_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(10s, std::bind(&RosoutSerialMonitor::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "rosout_serial_monitor started");
    }

private:
    void rosout_callback(const rcl_interfaces::msg::Log::SharedPtr msg)
    {
        // detect serial related errors in logs
        const std::string &m = msg->msg;
        bool match = false;
        if (m.find("串口读取异常") != std::string::npos) match = true;
        if (m.find("串口写入异常") != std::string::npos) match = true;
        if (m.find("无法打开端口") != std::string::npos) match = true;
        if (m.find("serial::IOException") != std::string::npos) match = true;
        if (m.find("IO Exception") != std::string::npos) match = true;

        if (!match) return;

        auto now = std::chrono::steady_clock::now();
        auto since = std::chrono::duration_cast<std::chrono::seconds>(now - last_restart_time_).count();
        if (since < 5) {
            RCLCPP_WARN(this->get_logger(), "Detected serial error but in cooldown (%.fs), skipping restart", (double)since);
            return;
        }

        RCLCPP_ERROR(this->get_logger(), "Detected serial error in /rosout: '%s' — restarting trans_node", m.c_str());

        // kill existing trans_node
        std::system("pkill -f 'ros2 run decision trans_node' || true");

        // small pause
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        // start trans_node with sourced workspace
        // use bash -lc to source the workspace then run node in background
        std::string cmd = "bash -lc 'cd /home/doublegun/sen_xin && source install/setup.bash && nohup ros2 run decision trans_node >/dev/null 2>&1 &'";
        int ret = std::system(cmd.c_str());
        if (ret == -1) {
            RCLCPP_ERROR(this->get_logger(), "Failed to launch trans_node (system call error)");
        } else {
            RCLCPP_INFO(this->get_logger(), "Restart command issued for trans_node");
        }

        last_restart_time_ = std::chrono::steady_clock::now();
    }

    void timer_callback()
    {
        // periodic status
        RCLCPP_DEBUG(this->get_logger(), "rosout_serial_monitor alive, last restart %ld seconds ago",
            std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - last_restart_time_).count());
    }

    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_restart_time_{std::chrono::steady_clock::time_point::min()};
};

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RosoutSerialMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
