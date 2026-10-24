#include "rm_behavior_tree/plugins/condition/restores_health.hpp"
#include <behaviortree_cpp/bt_factory.h>


namespace rm_behavior_tree
{
    IfRestoresHealthCondition::IfRestoresHealthCondition(
        const std::string & name, const BT::NodeConfig & config)
        : BT::SimpleConditionNode(name, std::bind(&IfRestoresHealthCondition::IfRestoresHealth, this), config)
    {
    }

    BT::NodeStatus IfRestoresHealthCondition::IfRestoresHealth()
    {
        // auto both_camp_hp = getInput<rm_decision_interfaces::msg::BothCampHp>("both_camp_hp");  //real
        auto both_camp_hp = getInput<std::uint16_t>("own_sentry_hp");   //sim
        auto sentry_blood_threshold = getInput<int>("sentry_blood_threshold");
        auto sentry_full_blood = getInput<int>("sentry_full_blood");

        //real
        // if(!both_camp_hp || !sentry_blood_threshold)
        // {
        //     setOutput<std::uint8_t>("if_first_zero", if_first_zero);
        //     return BT::NodeStatus::FAILURE;
        // }
        // if(both_camp_hp.value().own_sentry_hp <= sentry_blood_threshold.value())
        // {
        //     if_first_zero = 1;
        //     setOutput<std::uint8_t>("if_first_zero", if_first_zero);
        //     IfRestoresHealthCondition::flag = 1;
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else if(IfRestoresHealthCondition::flag == 1 && both_camp_hp.value().own_sentry_hp < sentry_full_blood.value())
        // {
        //     if_first_zero = 1;
        //     setOutput<std::uint8_t>("if_first_zero", if_first_zero);
        //     return BT::NodeStatus::SUCCESS;
        // }
        // else{
        //     IfRestoresHealthCondition::flag = 0;
        //     setOutput<std::uint8_t>("if_first_zero", if_first_zero);
        //     return BT::NodeStatus::FAILURE;
        // }   //real
        //sim
        if(!both_camp_hp || !sentry_blood_threshold)
        {
            setOutput<std::uint8_t>("if_first_zero", if_first_zero);
            return BT::NodeStatus::FAILURE;
        }

        // 如果当前血量为 0，立即标记为已降为 0（只要发生过一次），并输出到黑板
        if (both_camp_hp.value() == 0) {
            if_first_zero = 1;
            IfRestoresHealthCondition::flag = 1;
            setOutput<std::uint8_t>("if_first_zero", if_first_zero);
            std::cout << "[IfRestoresHealth] hp is 0, set if_first_zero=1" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        if(both_camp_hp.value() <= sentry_blood_threshold.value())
        {
            IfRestoresHealthCondition::flag = 1;
            setOutput<std::uint8_t>("if_first_zero", if_first_zero);
            // std::cout << "[IfRestoresHealth] hp(" << both_camp_hp.value() << ") <= blood_threshold(" << sentry_blood_threshold.value()
            //   << "), 没血了" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else if(IfRestoresHealthCondition::flag == 1 && both_camp_hp.value() < sentry_full_blood.value())
        {
            if(both_camp_hp.value() == 0){
                if_first_zero = 1;
                std::cout << "[IfRestoresHealth] hp(" << both_camp_hp.value() << ") <= blood_threshold(" << sentry_blood_threshold.value()
                  << "), 已置一" << std::endl;
            }
            setOutput<std::uint8_t>("if_first_zero", if_first_zero);
            // std::cout << "[IfRestoresHealth] hp(" << both_camp_hp.value() << ") <= blood_threshold(" << sentry_blood_threshold.value()
            //   << "), 在回血" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        else{
            IfRestoresHealthCondition::flag = 0;
            setOutput<std::uint8_t>("if_first_zero", if_first_zero);
            return BT::NodeStatus::FAILURE;
        }    //sim
    }
    
}

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<rm_behavior_tree::IfRestoresHealthCondition>("IfRestoresHealth");
}