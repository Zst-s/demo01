#pragma once

#include <cmath>
#include <rog_map/projection_layer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace sentry_terrain_bridge {

// Each point is one classified XY column, not a clearance-certified body box.
// z / ceiling_z retain the occupied span endpoints for subsequent frame handling.
inline sensor_msgs::msg::PointCloud2 hollowStructureCloud(
    const rog_map::ProjectionLayer &layer, const std::string &frame,
    const builtin_interfaces::msg::Time &stamp) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(5,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "ceiling_z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "resolution", 1, sensor_msgs::msg::PointField::FLOAT32);
  const auto selected = [](const rog_map::CellData &cell) {
    return cell.raw_reason == rog_map::ProjectionClassReason::HOLLOW_TUNNEL &&
        cell.raw_type == rog_map::CellType::PASSABLE &&
        cell.type == rog_map::CellType::PASSABLE && cell.mask == 1 &&
        std::isfinite(cell.occupied_z_min_abs) &&
        std::isfinite(cell.occupied_z_max_abs) &&
        cell.occupied_z_max_abs > cell.occupied_z_min_abs;
  };
  size_t count = 0;
  for (const auto &cell : layer.cells()) { if (selected(cell)) { ++count; } }
  modifier.resize(count);
  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"),
      z(cloud, "z"), ceiling(cloud, "ceiling_z"), resolution(cloud, "resolution");
  for (size_t i = 0; i < layer.cells().size(); ++i) {
    const auto &cell = layer.cells()[i];
    if (!selected(cell)) { continue; }
    *x = layer.origin().x() + (i % layer.width() + 0.5) * layer.resolution();
    *y = layer.origin().y() + (i / layer.width() + 0.5) * layer.resolution();
    *z = cell.occupied_z_min_abs;
    *ceiling = cell.occupied_z_max_abs;
    *resolution = layer.resolution();
    ++x; ++y; ++z; ++ceiling; ++resolution;
  }
  return cloud;
}
}  // namespace sentry_terrain_bridge
