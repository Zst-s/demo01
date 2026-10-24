#include "rm_behavior_tree/plugins/action/sub_enemydetected.hpp"
#include <iostream>


namespace rm_behavior_tree
{
    SubEnemyDetectedAction::SubEnemyDetectedAction(
        const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
        : BT::RosTopicSubNode<std_msgs::msg::UInt8>(name,config,params)
    {
    }
    
    BT::NodeStatus SubEnemyDetectedAction::onTick(
        const std::shared_ptr<std_msgs::msg::UInt8> & last_msg)
    {
        if(last_msg)
        {
            setOutput<std::uint8_t>("enemy_detected", last_msg->data);
            // std::cout << "[SubEnemyDetected] enemy_detected(" << static_cast<int>(last_msg->data) << ")" << std::endl;
        }else{
            setOutput<std::uint8_t>("enemy_detected", 0);
            // std::cout << "[SubEnemyDetected] enemy_detected(0) [no message]" << std::endl;
        }

        return BT::NodeStatus::SUCCESS;
    }
}

#include <behaviortree_ros2/plugins.hpp>
CreateRosNodePlugin(rm_behavior_tree::SubEnemyDetectedAction, "SubEnemyDetected");