#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <iostream>
#include <rclcpp_action/rclcpp_action.hpp>
#include "base_interfaces_demo/msg/robot.hpp"
#include "base_interfaces_demo/msg/control.hpp"
#include "rm_decision_interfaces/msg/both_camp_hp.hpp"
#include "rm_decision_interfaces/msg/enemy_detected.hpp"
#include "rm_decision_interfaces/msg/robot_status.hpp"
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <cmath>
// #include "rcl_interfaces/msg/log.hpp"

using namespace std;
using std::placeholders::_1;

#define sBUFFERSIZE 30                   // send buffer size 串口发送缓存长度
#define rBUFFERSIZE 30                   // receive buffer size 串口接收缓存长度
uint8_t s_buffer[sBUFFERSIZE];           // 发送缓存
unsigned char r_buffer[rBUFFERSIZE];     // 接收缓存
unsigned char after_buffer[rBUFFERSIZE]; // 处理后的数组

// 联合体,用于浮点数与16进制的快速转换
typedef union
{
    unsigned char cvalue[4];
    float fvalue;
} float_union;

// 创建一个serial类
serial::Serial ser;

// 保护串口操作的互斥量，防止并发读写导致异常未捕获
static std::mutex ser_mutex;

// 全局串口状态，供主循环与异常处理共享
static std::atomic<bool> serial_open(false);

// 流式接收缓存，用于粘包/拆包
// (moved recv buffer into the SerialChassis instance for thread-safety)

int arrival;
// 姿态：ATTACK=1 / DEFENSE=2 / MOVE=3，无 0 值。导航模式下 rm_bt 不运行、无 /target_stance 发布者，
// 若保持 0 底盘收不到合法姿态而拒动。默认 MOVE(3) 保证导航模式可直接行驶。
uint8_t target_stance = 3;

// 导航就绪标志：底盘需 nav_ready=1 且 target_stance 合法才接收速度。导航模式下底盘只响应
// nav2 的 /cmd_vel（nav2 未启动时本就无速度），rosout_node 的日志检测又不可靠，故默认置 1。
int nav_ready = 1;
int super_ele = 0;
int if_qifu = 0;
int intensify = 0;

class SerialChassis : public rclcpp::Node
{
public:
    SerialChassis() : Node("serial_chassis_node")
    {
        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        command_timeout_ = declare_parameter<double>("command_timeout", 0.25);
        if (!std::isfinite(command_timeout_) || command_timeout_ <= 0 || command_timeout_ > 1.0)
            throw std::invalid_argument("command_timeout must be in (0, 1] seconds");
        target_stance_sub = this->create_subscription<std_msgs::msg::UInt8>("/target_stance", 10, std::bind(&SerialChassis::target_stance_callback, this, _1));
        super_ele_sub = this->create_subscription<std_msgs::msg::UInt8>("/super_ele", 10, std::bind(&SerialChassis::super_ele_callback, this, _1));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&SerialChassis::cmd_vel_callback, this, _1));
        if_qifu_sub = this->create_subscription<std_msgs::msg::UInt8>("/if_qifu", 10, std::bind(&SerialChassis::if_qifu_callback, this, _1));
        intensify_sub = this->create_subscription<std_msgs::msg::UInt8>("/intensify", 10, std::bind(&SerialChassis::intensify_callback, this, _1));

        RobotStatus_pub = this->create_publisher<rm_decision_interfaces::msg::RobotStatus>("/robot_status", 10);
        enemy_detected_pub = this->create_publisher<std_msgs::msg::UInt8>("/enemy_detected", 10);
        game_status_pub = this->create_publisher<std_msgs::msg::UInt8>("/game_status", 10);
        tree_num_pub = this->create_publisher<std_msgs::msg::UInt8>("/tree_num", 10);
        outpost_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/outpost_if", 10);
        fort_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/fort_if", 10);
        midhigh_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/midhigh_if", 10);
        remain_time_pub = this->create_publisher<std_msgs::msg::UInt16>("/remain_time", 10);

        nav_ready_sub = this->create_subscription<std_msgs::msg::UInt8>("/nav_ready", 10, std::bind(&SerialChassis::nav_ready_callback, this, _1));


        mid_if_pub = this->create_publisher<std_msgs::msg::UInt8>("/mid_if", 10);        //RMUL
        both_camp_hp_pub = this->create_publisher<rm_decision_interfaces::msg::BothCampHp>("/both_camp_hp", 10);

    // 创建定时器，周期为10毫秒（原来为1ms，过于频繁会导致大量发送日志）
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SerialChassis::timer_callback, this));

        // 保护代码
        try
        {
            ser.setPort(serial_port_);
            ser.setBaudrate(115200);     // 设置串口通信的波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(10);
            ser.setTimeout(to);
            ser.open(); // 打开串口
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开端口！！！ 将在运行时重试打开串口");
            // Do not shutdown here; allow main loop to attempt reconnects
        }

        // 判断是否初始化成功（如果未打开，不在此处退出，主循环会尝试重连）
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "串行端口已初始化");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "串口未打开，构造时将不退出，主循环会重试打开串口");
        }

        // start background reader thread
        running_.store(true);
        read_thread_ = std::thread(&SerialChassis::read_loop, this);
    }

    ~SerialChassis()
    {
        // stop background thread and clean up
        running_.store(false);
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        // Best effort only: MCU must independently stop when this process dies.
        data_pack(geometry_msgs::msg::Twist());
        try {
            std::lock_guard<std::mutex> lk(ser_mutex);
            if (ser.isOpen()) ser.close();
        } catch (...) {}
    }

    // background reader thread
    void read_loop()
    {
        using namespace std::chrono_literals;
        while (rclcpp::ok() && running_.load()) {
            // ensure serial is open
            bool connected;
            { std::lock_guard<std::mutex> lk(ser_mutex); connected = ser.isOpen(); }
            if (!connected) {
                discard_command_.store(true);
                try {
                    std::lock_guard<std::mutex> lk(ser_mutex);
                    ser.setPort(serial_port_);
                    ser.setBaudrate(115200);
                    serial::Timeout to = serial::Timeout::simpleTimeout(10);
                    ser.setTimeout(to);
                    ser.open();
                    if (ser.isOpen()) {
                        RCLCPP_INFO(this->get_logger(), "串口已打开（read_loop 恢复连接）");
                    }
                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "尝试在 read_loop 中打开串口失败: %s", e.what());
                    std::this_thread::sleep_for(200ms);
                    continue;
                }
            }

            // Never hold the serial mutex waiting for a byte: that blocks stop commands.
            try {
                std::vector<unsigned char> temp(256);
                size_t nread = 0;
                {
                    std::lock_guard<std::mutex> lk(ser_mutex);
                    const auto available = ser.available();
                    if (available > 0)
                        nread = ser.read(temp.data(), std::min(available, size_t(256)));
                }
                if (nread > 0) {
                    std::lock_guard<std::mutex> lk(recv_mutex_);
                    recv_buffer_.insert(recv_buffer_.end(), temp.begin(), temp.begin() + nread);
                }
            } catch (const std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "串口读取异常 (read_loop): %s", e.what());
                try { std::lock_guard<std::mutex> lk(ser_mutex); ser.close(); } catch (...) {}
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            // process frames from recv_buffer_
            {
                std::lock_guard<std::mutex> lk(recv_mutex_);
                // look for full frames
                while (recv_buffer_.size() >= rBUFFERSIZE) {
                    auto it = std::find(recv_buffer_.begin(), recv_buffer_.end(), 0xA5);
                    if (it == recv_buffer_.end()) {
                        // no header, drop everything
                        recv_buffer_.clear();
                        break;
                    }
                    size_t idx = std::distance(recv_buffer_.begin(), it);
                    if (recv_buffer_.size() - idx < rBUFFERSIZE) {
                        // incomplete frame
                        break;
                    }
                    unsigned char frame[rBUFFERSIZE];
                    for (size_t i = 0; i < rBUFFERSIZE; ++i) frame[i] = recv_buffer_[idx + i];
                    // erase consumed bytes
                    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + idx + rBUFFERSIZE);
                    // process extracted frame (publish etc.)
                    process_frame(frame);
                }
            }

            std::this_thread::sleep_for(2ms);
        }
    }

    void process_frame(const unsigned char *new_r_buffer)
    {
        // 控制打印频率：只有每 print_every_ 帧打印一次所有字节，避免日志量过大
        int count = frame_counter_.fetch_add(1, std::memory_order_relaxed) + 1;
        bool printed_group = false;
        // 先计算血量，以便在分组打印中先输出血量行
        uint16_t current_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        if (count % print_every_ == 0) {
            printed_group = true;
            RCLCPP_INFO(this->get_logger(), "-----------------------------------------------");
            // 先打印血量行
            RCLCPP_INFO(this->get_logger(), "血量发送为: %d", current_hp);
            // 再打印每字节内容
            for (int i = 0; i < rBUFFERSIZE; ++i) {
                RCLCPP_INFO(this->get_logger(), "r%d发送为: %d", i, new_r_buffer[i]);
            }
        }

        // Mirror the processing logic from main() for a single extracted frame
        uint16_t own_sentry_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        uint8_t current_stance = (uint8_t)new_r_buffer[7];
        rm_decision_interfaces::msg::BothCampHp both_camp_hp_msg;
        both_camp_hp_msg.own_sentry_hp = own_sentry_hp;
        both_camp_hp_msg.current_stance = current_stance;
        both_camp_hp_pub->publish(both_camp_hp_msg);

    // 血量信息已在上面计算（用于分组打印）
        rm_decision_interfaces::msg::RobotStatus robotstatus_msg;
        robotstatus_msg.current_hp = current_hp;
        robotstatus_msg.outpost_hp = (uint16_t)new_r_buffer[1] << 8 | (uint16_t)new_r_buffer[2];
        robotstatus_msg.en_outpost_hp = (uint16_t)new_r_buffer[5] << 8 | (uint16_t)new_r_buffer[6];
        robotstatus_msg.current_stance = (uint8_t)new_r_buffer[7];
        robotstatus_msg.bullets_fired = (uint16_t)new_r_buffer[12] << 8 | (uint16_t)new_r_buffer[13];
        robotstatus_msg.remain_gold = (uint16_t)new_r_buffer[14] << 8 | (uint16_t)new_r_buffer[15];
        robotstatus_msg.fort_fired = (uint16_t)new_r_buffer[16] << 8 | (uint16_t)new_r_buffer[17];
        robotstatus_msg.resur_gold = (uint16_t)new_r_buffer[18] << 8 | (uint16_t)new_r_buffer[19];
        robotstatus_msg.ifoutpost = (uint8_t)new_r_buffer[25];    //是否识别敌方前哨站
        robotstatus_msg.in_defense_re_time = (uint8_t)new_r_buffer[27];     //强化防御剩余时间
        RobotStatus_pub->publish(robotstatus_msg);

        std_msgs::msg::UInt8 fort_if_msg;
        fort_if_msg.data = (uint8_t)new_r_buffer[22];
        fort_if_pub->publish(fort_if_msg);

        // std_msgs::msg::UInt8 outpost_if_msg;
        // outpost_if_msg.data = (uint8_t)new_r_buffer[23];
        // outpost_if_pub->publish(outpost_if_msg);

        // std_msgs::msg::UInt8 midhigh_if_msg;
        // midhigh_if_msg.data = (uint8_t)new_r_buffer[24];
        // midhigh_if_pub->publish(midhigh_if_msg);

        std_msgs::msg::UInt16 remain_time_msg;
        remain_time_msg.data = (uint16_t)new_r_buffer[20] << 8 | (uint16_t)new_r_buffer[21];
        remain_time_pub->publish(remain_time_msg);

        uint8_t game_status = (uint8_t)new_r_buffer[9];
        std_msgs::msg::UInt8 game_status_msg;
        game_status_msg.data = game_status;
        game_status_pub->publish(game_status_msg);

        uint8_t enemy_detected_flag = (uint8_t)new_r_buffer[8];
        std_msgs::msg::UInt8 enemy_detected_msg;
        enemy_detected_msg.data = enemy_detected_flag;
        enemy_detected_pub->publish(enemy_detected_msg);

        uint8_t tree_num = (uint8_t)new_r_buffer[10];
        std_msgs::msg::UInt8 tree_num_msg;
        tree_num_msg.data = tree_num;
        tree_num_pub->publish(tree_num_msg);

        uint8_t mid_if = (uint8_t)new_r_buffer[11];
        std_msgs::msg::UInt8 mid_if_msg;
        mid_if_msg.data = mid_if;
        mid_if_pub->publish(mid_if_msg);

        // 非分组帧不打印血量，避免日志噪声；血量将在分组打印时一起输出

        if (game_status == 5) {
            std::system("shutdown now");
        }
    }


    void data_pack(const geometry_msgs::msg::Twist &cmd_vel)
    {
        // 定义X Y Z（Ang_V）角速度
        float_union Vx, Vy, Vz;
        // 将传入的速度话题信息重新赋值
        Vx.fvalue = cmd_vel.linear.x;
        Vy.fvalue = cmd_vel.linear.y;
        Vz.fvalue = cmd_vel.angular.z;

        memset(s_buffer, 0, sizeof(s_buffer));      // 清空发送缓存

        s_buffer[0] = 0xA5;     
        s_buffer[1] = 9;        

        // 数据打包
        s_buffer[2] = Vx.cvalue[0];
        s_buffer[3] = Vx.cvalue[1];
        s_buffer[4] = Vx.cvalue[2];
        s_buffer[5] = Vx.cvalue[3];
        // Vy
        s_buffer[6] = Vy.cvalue[0];
        s_buffer[7] = Vy.cvalue[1];
        s_buffer[8] = Vy.cvalue[2];
        s_buffer[9] = Vy.cvalue[3];
        // Vz
        s_buffer[10] = Vz.cvalue[0];
        s_buffer[11] = Vz.cvalue[1];
        s_buffer[12] = Vz.cvalue[2];
        s_buffer[13] = Vz.cvalue[3];

        // s_buffer[14] = arrival;

        s_buffer[14] = target_stance;

        s_buffer[15] = nav_ready;  // 导航就绪标志

        s_buffer[16] = super_ele;  // 是否开超电

        s_buffer[17] = if_qifu;    // 起伏路段标志
        s_buffer[18] = intensify;  // 是否强化模式

        // 将打包好的数据发送（带锁和异常保护）
        std::lock_guard<std::mutex> lk(ser_mutex);
        if (!ser.isOpen()) {
            discard_command_.store(true);
            RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000, "串口未打开，跳过发送");
            return;
        }
        try {
            ser.write(s_buffer, sBUFFERSIZE);
            RCLCPP_DEBUG(this->get_logger(), "发送成功");
        } catch (const std::exception &e) {
            discard_command_.store(true);
            RCLCPP_ERROR(this->get_logger(), "串口写入异常: %s", e.what());
            // 尝试关闭串口以触发上层重连逻辑
            try { ser.close(); } catch (...) {}
        }
    }


    // target_stance话题的回调函数
    void target_stance_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        target_stance = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received target_stance data: %d", target_stance);
        new_target_stance_received = true;
    }

    // super_ele话题的回调函数
    void super_ele_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        super_ele = msg->data;
    }

    void if_qifu_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        if_qifu = msg->data;
    }

    void intensify_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        intensify = msg->data;
    }

    // nav_ready话题的回调函数，监控导航就绪状态
    void nav_ready_callback(const std_msgs::msg::UInt8::SharedPtr msg)
    {
        nav_ready = 0;
        nav_ready = msg->data;
        nav_ready_received = true;
    }


    // 订阅cmd_vel话题的回调函数,用于显示速度以及角速度
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel)
    {
        const auto &v = *cmd_vel;
        const bool valid = std::isfinite(v.linear.x) && std::isfinite(v.linear.y) &&
            std::isfinite(v.angular.z) && v.linear.z == 0 && v.angular.x == 0 && v.angular.y == 0;
        cmd_vel_buffer = valid ? v : geometry_msgs::msg::Twist();
        last_command_ = std::chrono::steady_clock::now();
        new_cmd_vel_received = true;
    }

    // 定时器回调函数
    void timer_callback()
    {
        const double age = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - last_command_).count();
        if (discard_command_.exchange(false) || age > command_timeout_ || nav_ready == 0)
            cmd_vel_buffer = geometry_msgs::msg::Twist();
        data_pack(cmd_vel_buffer);
        new_cmd_vel_received = false;
        new_target_stance_received = false;
        nav_ready_received = false;
    }

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr target_stance_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr super_ele_sub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr if_qifu_sub;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr intensify_sub;

    rclcpp::Publisher<rm_decision_interfaces::msg::RobotStatus>::SharedPtr RobotStatus_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr enemy_detected_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_status_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr tree_num_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr outpost_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr fort_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr midhigh_if_pub;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_time_pub;

    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mid_if_pub;           //RMUL
    rclcpp::Publisher<rm_decision_interfaces::msg::BothCampHp>::SharedPtr both_camp_hp_pub;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist cmd_vel_buffer;
    std::string serial_port_;
    double command_timeout_ = 0.25;
    std::chrono::steady_clock::time_point last_command_{};
    std::atomic<bool> discard_command_{false};
    bool new_cmd_vel_received = false;
    bool new_target_stance_received = false;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_ready_sub;       // 订阅 /rosout 监控终端输出
    bool nav_ready_received = true;
    // recv buffer and thread for reading
    std::vector<unsigned char> recv_buffer_;
    std::mutex recv_mutex_;
    std::thread read_thread_;
    std::atomic<bool> running_{false};
    // 控制打印频率，避免过多日志
    std::atomic<int> frame_counter_{0};
    int print_every_ = 5; // 每5帧打印一次（可调整）

};

// 接收数据分析与校验
/*bool data_analysis(unsigned char *buffer)
{
    // 定义标志位 求和
    bool flag = true;
    int temp;
    int l = 0; // 强制拉回位
    // unsigned char csum;
    if ((buffer[0] == 0xA5))
    {
        flag = true; // 校验通过
    }
    else
    {
        // 去去年的土方法
        if (buffer[0] != 0xA5)
        {
            for (int b = 0; b < rBUFFERSIZE; b++) // 1
            {
                buffer[b] = buffer[b + 1];
            }
            if (buffer[0] != 0xA5)
            {
                for (int b = 0; b < rBUFFERSIZE; b++) // 2
                {
                    buffer[b] = buffer[b + 1];
                }
                if (buffer[0] != 0xA5)
                {
                    for (int b = 0; b < rBUFFERSIZE; b++) // 3
                    {
                        buffer[b] = buffer[b + 1];
                    }
                    if (buffer[0] != 0xA5)
                    {
                        for (int b = 0; b < rBUFFERSIZE; b++) // 4
                        {
                            buffer[b] = buffer[b + 1];
                        }
                        if (buffer[0] != 0xA5)
                        {
                            for (int b = 0; b < rBUFFERSIZE; b++) // 5
                            {
                                buffer[b] = buffer[b + 1];
                            }
                            if (buffer[0] != 0xA5)
                            {
                                for (int b = 0; b < rBUFFERSIZE; b++) // 6
                                {
                                    buffer[b] = buffer[b + 1];
                                }
                                if (buffer[0] != 0xA5)
                                {
                                    for (int b = 0; b < rBUFFERSIZE; b++) // 7
                                    {
                                        buffer[b] = buffer[b + 1];
                                    }
                                    if (buffer[0] != 0xA5)
                                    {
                                        for (int b = 0; b < rBUFFERSIZE; b++) // 8
                                        {
                                            buffer[b] = buffer[b + 1];
                                        }
                                        if (buffer[0] != 0xA5)
                                        {
                                            for (int b = 0; b < rBUFFERSIZE; b++) // 9
                                            {
                                                buffer[b] = buffer[b + 1];
                                            }
                                            if (buffer[0] != 0xA5)
                                            {
                                                for (int b = 0; b < rBUFFERSIZE; b++) // 10
                                                {
                                                    buffer[b] = buffer[b + 1];
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return flag; // 返回校验是否通过
    // ROS_INFO("%d\n",flag);
}*/

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SerialChassis>();
    // Let ROS spin; background thread handles serial reads and publishes
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
