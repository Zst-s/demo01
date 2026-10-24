#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "base_interfaces_demo/msg/robot.hpp"
#include "base_interfaces_demo/msg/control.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"


#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <iostream>

#include <fstream>

using std::placeholders::_1;
using namespace std::chrono_literals;
using base_interfaces_demo::msg::Robot;
using base_interfaces_demo::msg::Control;


class SendGoal : public rclcpp::Node
{
public:
    SendGoal() : Node("send_goal"), singel(0)       //初始化singel为 0
    {
        flag_sub = this->create_subscription<Robot>("/bleed", 100, std::bind(&SendGoal::flag_callback, this, _1));
        
        
        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);       //发送目标位姿 带时间戳
        arrival_pub = this->create_publisher<std_msgs::msg::Int32>("/arrival", 10);     //发送到达状态
        singel_pub = this->create_publisher<Control>("/robot_singel", 10);
        
        //
        tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&SendGoal::publish_singel, this));


    }

    void flag_callback(const Robot::SharedPtr robot)
    {
        flag_real = robot->flag;

        if(flag_real == 0){
            flag_real = 1;
        }

        RCLCPP_INFO(this->get_logger(), "状态为>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>: %d", flag_real);

    }



//初始位置
    void send(const float &x, const float &y, const float&w)
    {
        geometry_msgs::msg::PoseStamped send_pose;
        send_pose.header.frame_id = "map"; 
        send_pose.pose.position.x = x;
        send_pose.pose.position.y = y;
        send_pose.pose.position.z = 0.0;
        send_pose.pose.orientation.x = 0.0;
        send_pose.pose.orientation.y = 0.0;
        send_pose.pose.orientation.z = 0.0;
        send_pose.pose.orientation.w = w;
        goal_pub->publish(send_pose);       //  /goal_pose
        RCLCPP_INFO(this->get_logger(), "Published goal: x=%.2f, y=%.2f, w=%.2f", x, y, w);
    }


//距离终点位置
    void checkArrival(const float &target_x, const float &target_y)
    {
        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePoint());
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        float dx = fabs(transform.transform.translation.x - target_x);
        float dy = fabs(transform.transform.translation.y - target_y);

        std_msgs::msg::Int32 arrival_msg;
        if (dx <= 0.3 && dy <= 0.3)
        {
            arrival_msg.data = 1;
            RCLCPP_INFO(this->get_logger(), "到了");
        }
        else
        {
            arrival_msg.data = 0;
            RCLCPP_INFO(this->get_logger(), "没到");
        }
        arrival_pub->publish(arrival_msg);      //  /arrival

        // checkZandWrite(transform);
    }


//重定位
    // void checkZandWrite(const geometry_msgs::msg::TransformStamped &transform)
    // {
    //     std::string translation_path = "/home/doublegun/translation.txt";
    //     std::string rotation_path = "/home/doublegun/rotation.txt";

    //     if (transform.transform.translation.z < 0.2 && transform.transform.translation.z > -0.2)
    //     {
    //         std::ofstream trans_file(translation_path, std::ofstream::trunc);
    //         std::ofstream rot_file(rotation_path, std::ofstream::trunc);

    //         trans_file << "Translation: " 
    //             << "x=" << transform.transform.translation.x << ", "
    //             << "y=" << transform.transform.translation.y << ", "
    //             << "z=" << transform.transform.translation.z;
    //         rot_file << "Rotation: " 
    //             << "x=" << transform.transform.rotation.x << ", "
    //             << "y=" << transform.transform.rotation.y << ", "
    //             << "z=" << transform.transform.rotation.z << ", "
    //             << "w=" << transform.transform.rotation.w;

    //         trans_file.close();
    //         rot_file.close();
    //         RCLCPP_INFO(this->get_logger(), "singel = 0");
    //     }
    //     else
    //     {
    //         singel = 1;
    //         RCLCPP_INFO(this->get_logger(), "singel = 1");

    //     }

    // }

    void publish_singel()
    {
        Control control_msg;
        control_msg.singel = singel;

        singel_pub->publish(control_msg);       //  /robot_singel

        if (singel == 1)
        {
            RCLCPP_INFO(this->get_logger(), "singel=1, shutting down...");
            rclcpp::shutdown();
            system("ps aux | grep 'decision' | grep -v grep | awk '{print $2}' | xargs -r kill -9");
            exit(0); 
        }
    
    
    }


    char flag_real;
    int singel;
    


    rclcpp::Subscription<Robot>::SharedPtr flag_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr arrival_pub;
    rclcpp::Publisher<Control>::SharedPtr singel_pub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendGoal>();

     // 程序开始时睡眠15秒
    rclcpp::sleep_for(std::chrono::seconds(15));

    // const float x_1 = 6.68; 
    // const float y_1 = -0.412;

    // const float x_2 = 9.74; 
    // const float y_2 = -0.6;

    // const float x_3 = 13.1; 
    // const float y_3 = -0.893;

    // const float x_4 = 16.8; 
    // const float y_4 = -1.08;

    // const float x_5 = 19.6; 
    // const float y_5 = -1.36;
    
    const float x_1 = 17.1;
    const float y_1 = 0.2;
    const float w_1 = 1.0;

    const float x_2= 1.39; 
    const float y_2= -5.66;//自家前哨站
    const float w_2 = 1.0;

    const float x_3=  16.15; 
    const float y_3 = -6.12;//对面大资
    const float w_3 = 1.0;

    const float x_4 = 7.80; 
    const float y_4 = -2.48;//对面前哨站
    const float w_4 = 1.0;

    const float x_6 = 11.5; 
    const float y_6 = -2.69;//对面的哨兵巡逻区
    const float w_6 = 1.0;

    const float x_5 = 11.05; 
    const float y_5 = -2.69;//自家环高狗洞下 
    const float w_5 = 1.0;
    
    const float x_7 = 3.80;
    const float y_7 = -3.05;//原点
    const float w_7 = 1.0;

    const float x_8 = 5.30;
    const float y_8 = -2.15;//对面落风坡
    const float w_8 = 1.0;


    rclcpp::WallRate loop_rate(10);

    while (rclcpp::ok())
    {
        switch (node->flag_real)
        {
            case 0:
                RCLCPP_INFO(node->get_logger(), "不动");
                break;
            case 1:
                node->send(x_1, y_1, w_1);
                node->checkArrival(x_1, y_1);
                break;
             case 2:
                 node->send(x_2, y_2, w_2);
                 node->checkArrival(x_2, y_2);
                 break;
             case 3:
                 node->send(x_3, y_3, w_3);
                 node->checkArrival(x_3, y_3);
                 break;
             case 4:
                 node->send(x_4, y_4, w_4);
                 node->checkArrival(x_4, y_4);
                 break;
             case 5:
                 node->send(x_5, y_5, w_5);
                 node->checkArrival(x_5, y_5);
                 break;
             case 6:
                 node->send(x_6, y_6, w_6);
                 node->checkArrival(x_6, y_6);
                 break;
            case 7:
                node->send(x_7, y_7, w_7);
                 node->checkArrival(x_7, y_7);
                 break;
             case 8:
                node->send(x_8, y_8, w_8);
                 node->checkArrival(x_8, y_8);
                 break;
            default:
                RCLCPP_WARN(node->get_logger(), "未知的标志位");
                break;
        }

        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}