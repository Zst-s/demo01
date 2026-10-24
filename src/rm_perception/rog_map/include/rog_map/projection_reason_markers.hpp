#pragma once
#include <array>
#include <string>
#include <rog_map/projection_layer.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace rog_map {
// 原始分类原因不等于最终代价：迟滞、补洞等后处理可能改变最终类型。
inline visualization_msgs::msg::MarkerArray projectionReasonMarkers(
  const ProjectionLayer & layer, const std::string & frame,
  const builtin_interfaces::msg::Time & stamp)
{
  using Marker = visualization_msgs::msg::Marker;
  const std::array<std::string, 6> names = {
    "0_unknown", "1_empty", "2_thin_surface", "3_solid_wall",
    "4_hollow_candidate", "5_ambiguous"};
  const std::array<std::array<float, 3>, 6> colors = {{
    {0.45F, 0.45F, 0.45F}, {0.15F, 0.65F, 1.0F}, {0.2F, 0.9F, 0.3F},
    {1.0F, 0.15F, 0.15F}, {1.0F, 0.8F, 0.0F}, {0.8F, 0.2F, 0.9F}}};
  visualization_msgs::msg::MarkerArray out;
  for (size_t i = 0; i < names.size(); ++i) {
    Marker marker;
    marker.header.frame_id = frame; marker.header.stamp = stamp;
    marker.ns = names[i]; marker.id = static_cast<int>(i);
    marker.type = Marker::CUBE_LIST; marker.action = Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker.scale.y = layer.resolution() * 0.9;
    marker.scale.z = 0.025;
    marker.color.r = colors[i][0]; marker.color.g = colors[i][1];
    marker.color.b = colors[i][2]; marker.color.a = 0.8F;
    marker.lifetime.sec = 2;
    out.markers.push_back(marker);
  }
  for (size_t i = 0; i < layer.cells().size(); ++i) {
    const auto reason = static_cast<size_t>(layer.cells()[i].raw_reason);
    if (reason >= names.size() || layer.width() <= 0) { continue; }
    geometry_msgs::msg::Point point;
    point.x = layer.origin().x() + (i % layer.width() + 0.5) * layer.resolution();
    point.y = layer.origin().y() + (i / layer.width() + 0.5) * layer.resolution();
    point.z = 0.035;  // 原因投影图，不伪装成三维障碍高度。
    out.markers[reason].points.push_back(point);
  }
  for (auto & marker : out.markers) {
    if (marker.points.empty()) { marker.action = Marker::DELETE; }
  }
  Marker legend;
  legend.header.frame_id = frame; legend.header.stamp = stamp;
  legend.ns = "legend"; legend.id = 99; legend.type = Marker::TEXT_VIEW_FACING;
  legend.pose.orientation.w = 1.0;
  legend.pose.position.x = layer.origin().x() + 2.0;
  legend.pose.position.y = layer.origin().y() - 0.5;
  legend.pose.position.z = 0.3;
  legend.scale.z = 0.16;
  legend.color.r = legend.color.g = legend.color.b = legend.color.a = 1.0F;
  legend.lifetime.sec = 2;
  legend.text = "RAW REASONS (not clearance permission)\n"
    "Gray: unknown | Blue: empty | Green: thin surface\n"
    "Red: solid wall | Yellow: hollow candidate | Purple: ambiguous";
  out.markers.push_back(legend);
  return out;
}
}  // namespace rog_map
