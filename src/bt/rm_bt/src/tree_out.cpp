#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <thread>

using namespace std;

class BtMonitor : public rclcpp::Node
{
public:
    BtMonitor() : Node("bt_monitor_node"), start_time_(std::chrono::steady_clock::now())
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&BtMonitor::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "bt_monitor_node 已启动，监控 tree_exec_node 进程...");
    }

private:
    int get_process_count()
    {
        // 排除 bt_monitor 和 goal_pose_bridge，只匹配 tree_exec_node
        FILE* pipe = popen("pgrep -c -f 'tree_exec_node' 2>/dev/null || echo 0", "r");
        if (!pipe) return -1;
        
        char buffer[32] = {0};
        if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            int count = atoi(buffer);
            pclose(pipe);
            return count;
        }
        pclose(pipe);
        return -1;
    }

    void timer_callback()
    {
        const auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_).count();
        
        // 启动后等待5秒
        if (elapsed < 5)
        {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                                 "等待启动... (%ld秒)", elapsed);
            return;
        }

        int count = get_process_count();
        bool running = (count > 1);
        
        RCLCPP_INFO(this->get_logger(), "检查: count=%d, seen=%d", count, process_seen_);
        
        if (running)
        {
            if (!process_seen_)
            {
                process_seen_ = true;
                RCLCPP_INFO(this->get_logger(), "检测到 tree_exec_node 进程，开始监控");
            }
        }
        else
        {
            if (process_seen_)
            {
                RCLCPP_ERROR(this->get_logger(), "tree_exec_node 崩溃！正在重启...");
                
                std::system("pkill -9 -f 'bt_tree.launch.py'");
                std::system("pkill -9 -f 'goal_pose_bridge'");
                
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
                std::system("gnome-terminal -- bash -c '"
                            "source /home/doublegun/sen/install/setup.bash && "
                            "ros2 launch rm_bt bt_tree.launch.py; exec bash'");
                
                RCLCPP_INFO(this->get_logger(), "已在新终端重启");
                
                process_seen_ = false;
                start_time_ = std::chrono::steady_clock::now();
            }
            else
            {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000, 
                                     "等待 tree_exec_node 启动...");
            }
        }
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point start_time_;
    bool process_seen_ = false;
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BtMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
