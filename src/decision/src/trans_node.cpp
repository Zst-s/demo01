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

int arrival;
uint8_t target_stance;

int nav_ready = 0;
int super_ele = 0;
int if_qifu = 0;
int intensify = 0;

class SerialChassis : public rclcpp::Node
{
public:
    SerialChassis() : Node("serial_chassis_node")
    {
        target_stance_sub = this->create_subscription<std_msgs::msg::UInt8>("/target_stance", 10, std::bind(&SerialChassis::target_stance_callback, this, _1));
        super_ele_sub = this->create_subscription<std_msgs::msg::UInt8>("/super_ele", 10, std::bind(&SerialChassis::super_ele_callback, this, _1));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&SerialChassis::cmd_vel_callback, this, _1));
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

        // 创建定时器，周期为10毫秒
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&SerialChassis::timer_callback, this));

        // 保护代码
        try
        {
            ser.setPort("/dev/ttyACM0"); // 设置要打开的串口名称
            ser.setBaudrate(115200);     // 设置串口通信的波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open(); // 打开串口
        }
        catch (serial::IOException &e)
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开端口！！！");
            rclcpp::shutdown();
        }

        // 判断是否初始化成功
        if (ser.isOpen())
        {
            RCLCPP_INFO(this->get_logger(), "串行端口已初始化");
        }
        else
        {
            rclcpp::shutdown();
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

        // 将打包好的数据发送
        ser.write(s_buffer, sBUFFERSIZE);
        RCLCPP_INFO(this->get_logger(), "发送成功");
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
        // 存储最新的速度指令，并标记新指令已经接收
        cmd_vel_buffer = *cmd_vel;
        new_cmd_vel_received = true;
    }

    // 定时器回调函数
    void timer_callback()
    {
        // 如果有新的速度指令到来，就发送该速度指令
        if (new_cmd_vel_received || new_target_stance_received || nav_ready_received)
        {
            data_pack(cmd_vel_buffer);
            new_cmd_vel_received = false;
            new_target_stance_received = false;
            nav_ready_received = true;
            // rosout_received = false;
        }
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
    bool new_cmd_vel_received = false;
    bool new_target_stance_received = false;

    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr nav_ready_sub;       // 订阅 /rosout 监控终端输出
    bool nav_ready_received = true;

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
    bool serial_open = true;

    // 程序开始时睡眠15秒
    // rclcpp::sleep_for(std::chrono::seconds(25));

    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {
        if (serial_open == false)
        {
            ser.setPort("/dev/ttyUSB0"); // 设置要打开的串口名称
            ser.setBaudrate(115200);     // 设置串口通信的波特率
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser.setTimeout(to);
            ser.open(); // 打开串口
            serial_open = true;
            RCLCPP_INFO(node->get_logger(), "串口重新连接成功！");
        }
        else
        {
            try
            {
                ser.open();
            }
            catch (const std::exception &e)
            {
                std::cerr << e.what() << '\n';
            }
        }
        // if (!ser.isOpen())
        // {
        //     ser.open();
        // }
        // if (ser.available()) {
        //     //读取数据
        //     ser.read(r_buffer, rBUFFERSIZE);

        //     if(r_buffer[0] != 0xA5)
        //         continue;

        //     RCLCPP_INFO(node->get_logger(), "flag发送为: %d", r_buffer[2]);
        //     flag_now = r_buffer[2];

        //     base_interfaces_demo::msg::Robot robot;
        //     robot.flag = flag_now;

        //     // RCLCPP_INFO(node->get_logger(), "flag_now状态为: %d", flag_now);
        //     node->publishGoal(robot);

        //         //清空接收缓存
        //     memset(r_buffer, 0, rBUFFERSIZE);

        // }
        // else
        // {
        //     RCLCPP_WARN(node->get_logger(), "message error!");
        // }
        auto start_time = std::chrono::high_resolution_clock::now();
        size_t zj_num = ser.read(r_buffer, rBUFFERSIZE);
        auto end_time = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

        RCLCPP_INFO(node->get_logger(), "耗时：%ld ms", duration);
        RCLCPP_INFO(node->get_logger(), "字节数：%ld", zj_num);

        unsigned char new_r_buffer[rBUFFERSIZE];
        if (r_buffer[0] != 0xA5)
        {
            bool success = false;
            int many = sizeof(r_buffer) / sizeof(r_buffer[0]);
            RCLCPP_INFO(node->get_logger(), "强制拉回操作启动！");
            for (uint i = 0; i < many; i++) // 防串位用的
            {
                uint j = i + 1;
                if (j == many)
                {
                    j = j - many;
                }
                if (r_buffer[i] == 0xA5)
                {
                    success = true;
                    uint l = i;
                    for (uint k = 0; k < many; k++)
                    {
                        if (l == many)
                        {
                            l = l - many;
                        }
                        new_r_buffer[k] = r_buffer[l];
                        l++;
                    }
                }
            }
            if (!success)
            {
                RCLCPP_INFO(node->get_logger(), "Don't have header!");
                serial_open = false;
            }
        }
        else
        {
            for (int i = 0; i < sizeof(r_buffer) / sizeof(r_buffer[0]); i++)
            {
                new_r_buffer[i] = r_buffer[i];
            }
        }
        for (int i = 0; i < 29; i++)
        {
            RCLCPP_INFO(node->get_logger(), "r%d发送为: %d", i, new_r_buffer[i]);
        }

        RCLCPP_INFO(node->get_logger(), "-----------------------------------------------");
        // flag_now = new_r_buffer[2];

        //----------------------------------------------------------------------------------------------
        // 0                          1 --> 前哨站血量前8位         2 --> 前哨站血量后8位         3 --> 自身血量前8位                                           
        // 4 --> 自身血量后8位          5 --> 敌方前哨站血量前8位      6 --> 敌方前哨站血量后8位      7 --> 当前姿态                                            
        // 8 --> 是否识别到敌人         9 -->  游戏状态              10 --> 决策版本              11 --> （中央占领状态 RMUL用）                                           
        // 12 --> 剩余发弹量前8位       13 --> 剩余发弹量后8位        14 --> 剩余金币前8位          15 --> 剩余金币后8位                                                                                                                                                         
        // 16 --> 堡垒提供发弹量前8位    17 --> 堡垒提供发弹量后8位    18 --> 立即复活金币前8位       19 --> 立即复活金币后8位                                                                                             
        // 20 --> 剩余时间前8位         21 --> 剩余时间后8位         22 --> 堡垒占领状态           23 --> 前哨站占领状态
        // 24 --> 中央高地占领状态       25 --> 是否识别敌方前哨站     26 --> 强化进攻剩余时间        27 --> 强化防御剩余时间
        // 28 --> 强化移动剩余时间                                                                                                                                                                                                                                  
        //----------------------------------------------------------------------------------------------

        //rm_decision

        uint16_t own_sentry_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        uint8_t current_stance = (uint8_t)new_r_buffer[7];
        rm_decision_interfaces::msg::BothCampHp both_camp_hp_msg;
        both_camp_hp_msg.own_sentry_hp = own_sentry_hp;
        // both_camp_hp_msg.own_sentry_hp = 220;
        both_camp_hp_msg.current_stance = current_stance;
        node->both_camp_hp_pub->publish(both_camp_hp_msg);      //血量、姿态发布

        uint16_t current_hp = (uint16_t)new_r_buffer[3] << 8 | (uint16_t)new_r_buffer[4];
        RCLCPP_INFO(node->get_logger(), "血量发送为: %d", current_hp);
        rm_decision_interfaces::msg::RobotStatus robotstatus_msg;
        robotstatus_msg.current_hp = current_hp;                                //当前血量
        robotstatus_msg.outpost_hp = (uint16_t)new_r_buffer[1] << 8 | (uint16_t)new_r_buffer[2];    //前哨站血量
        robotstatus_msg.en_outpost_hp = (uint16_t)new_r_buffer[5] << 8 | (uint16_t)new_r_buffer[6];       //敌方前哨站血量
        robotstatus_msg.current_stance = (uint8_t)new_r_buffer[7];              //当前姿态
        robotstatus_msg.bullets_fired = (uint16_t)new_r_buffer[12] << 8 | (uint16_t)new_r_buffer[13];     //剩余发弹量
        robotstatus_msg.remain_gold = (uint16_t)new_r_buffer[14] << 8 | (uint16_t)new_r_buffer[15];     //剩余金币 
        robotstatus_msg.fort_fired = (uint16_t)new_r_buffer[16] << 8 | (uint16_t)new_r_buffer[17];     //堡垒发弹量
        robotstatus_msg.resur_gold = (uint16_t)new_r_buffer[18] << 8 | (uint16_t)new_r_buffer[19];    //复活金币
        robotstatus_msg.ifoutpost = (uint8_t)new_r_buffer[25];    //是否识别敌方前哨站
        robotstatus_msg.in_defense_re_time = (uint8_t)new_r_buffer[27];     //强化防御剩余时间
        node->RobotStatus_pub->publish(robotstatus_msg);      //robot_status发布

        std_msgs::msg::UInt8 fort_if_msg;
        fort_if_msg.data = (uint8_t)new_r_buffer[22];
        node->fort_if_pub->publish(fort_if_msg);              //堡垒占领状态发布

        // std_msgs::msg::UInt8 outpost_if_msg;
        // outpost_if_msg.data = (uint8_t)new_r_buffer[23];
        // node->outpost_if_pub->publish(outpost_if_msg);        //前哨站占领状态发布

        // std_msgs::msg::UInt8 midhigh_if_msg;
        // midhigh_if_msg.data = (uint8_t)new_r_buffer[24];
        // node->midhigh_if_pub->publish(midhigh_if_msg);        //中央高地占领状态发布

        std_msgs::msg::UInt16 remain_time_msg;
        remain_time_msg.data = (uint16_t)new_r_buffer[20] << 8 | (uint16_t)new_r_buffer[21];
        node->remain_time_pub->publish(remain_time_msg);      //剩余时间发布

        uint8_t game_status = (uint8_t)new_r_buffer[9];
        std_msgs::msg::UInt8 game_status_msg;
        game_status_msg.data = game_status;
        // game_status_msg.data = 4;
        node->game_status_pub->publish(game_status_msg);        //游戏状态发布

        uint8_t enemy_detected_flag = (uint8_t)new_r_buffer[8];
        std_msgs::msg::UInt8 enemy_detected_msg;
        enemy_detected_msg.data = enemy_detected_flag;
        node->enemy_detected_pub->publish(enemy_detected_msg);  //是否识别到敌人发布

        uint8_t tree_num = (uint8_t)new_r_buffer[10];
        std_msgs::msg::UInt8 tree_num_msg;
        tree_num_msg.data = tree_num;
        node->tree_num_pub->publish(tree_num_msg);              //决策版本发布

        // if(tree_num == 1)
        // {
        //     game_status_msg.data = 4;
        // }
        // node->game_status_pub->publish(game_status_msg);        //根据决策版本发布游戏状态

        //----------------------------------RMUL---------------------------------------------
        uint8_t mid_if = (uint8_t)new_r_buffer[11];
        std_msgs::msg::UInt8 mid_if_msg;
        mid_if_msg.data = mid_if;
        // mid_if_msg.data = 2;
        node->mid_if_pub->publish(mid_if_msg);              //中央占领状态发布
        //----------------------------------RMUL------------------------------------------------

        //rm_decision_end

        if(game_status == 5){                               //比赛结算时NUC关机
            std::system("shutdown now");
        }


        // 清空接收缓存
        // memset(r_buffer, 0, rBUFFERSIZE);
        rclcpp::spin_some(node);
        // loop_rate.sleep();
        ser.close();
    }

    rclcpp::shutdown();
    return 0;
}
