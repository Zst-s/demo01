#ifndef RM_BEHAVIOR_TREE__BTCONVERSIONS_HPP_
#define RM_BEHAVIOR_TREE__BTCONVERSIONS_HPP_

#include <behaviortree_cpp/basic_types.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

namespace BT
{
template <>
inline geometry_msgs::msg::PointStamped convertFromString<geometry_msgs::msg::PointStamped>(StringView str)
{
  const auto parts = BT::splitString(str, ',');
  if (parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input");
  }

  geometry_msgs::msg::PointStamped point;
  point.point.x = convertFromString<double>(parts[0]);
  point.point.y = convertFromString<double>(parts[1]);
  point.point.z = convertFromString<double>(parts[2]);
  return point;
}

template <>
inline geometry_msgs::msg::PoseStamped convertFromString(const StringView key)
{
  auto parts = splitString(key, ';');
  if (parts.size() != 7)
  {
    throw RuntimeError("invalid input)");
  }
  else
  {
    geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = convertFromString<double>(parts[0]);
    msg.pose.position.y = convertFromString<double>(parts[1]);
    msg.pose.position.z = convertFromString<double>(parts[2]);
    msg.pose.orientation.x = convertFromString<double>(parts[3]);
    msg.pose.orientation.y = convertFromString<double>(parts[4]);
    msg.pose.orientation.z = convertFromString<double>(parts[5]);
    msg.pose.orientation.w = convertFromString<double>(parts[6]);
    return msg;
  }
}
}  // namespace BT

#endif  // RM_BEHAVIOR_TREE__BTCONVERSIONS_HPP_