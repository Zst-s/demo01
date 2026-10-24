#ifndef RM_BEHAVIOR_TREE__BT_CONVERSIONS_STRINGTOINT_HPP_
#define RM_BEHAVIOR_TREE__BT_CONVERSIONS_STRINGTOINT_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <std_msgs/msg/int16.hpp>

namespace BT
{
    template<>
    inline std_msgs::msg::Int16 convertFromString<std_msgs::msg::Int16>(const StringView key)
    {
        std_msgs::msg::Int16 msg;
        try
        {
            msg.data = std::stoi(std::string(key));     // 将 StringView 转换为 std::string 后再转换为整数
        }
        catch (const std::invalid_argument & e)
        {
            throw BT::RuntimeError("Failed to convert string to Int16: " + std::string(e.what()));
        }
        return msg;
    }
}

#endif  // RM_BEHAVIOR_TREE__BT_CONVERSIONS_STRINGTOINT_HPP_