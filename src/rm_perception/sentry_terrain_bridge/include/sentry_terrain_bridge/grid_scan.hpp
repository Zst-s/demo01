#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace sentry_terrain_bridge {

// SLAM Toolbox 会无条件接收首帧，静止时后续扫描又受移动距离门限限制。
// 没有有效回波的首帧会产生零尺寸地图，因此等到真正观测到障碍再送入建图。
inline bool hasUsableMappingReturn(const sensor_msgs::msg::LaserScan &scan) {
  for (const auto range : scan.ranges) {
    if (std::isfinite(range) && range > scan.range_min &&
        range < scan.range_max) {
      return true;
    }
  }
  return false;
}

// 只把 ROGMap 明确判为墙的格子转换成扫描终点；未知区不虚构墙面。
inline sensor_msgs::msg::LaserScan projectGridToScan(
    const nav_msgs::msg::OccupancyGrid &grid, double sensor_x,
    double sensor_y, double sensor_yaw, const std::string &scan_frame,
    float range_min, float range_max, float angle_increment,
    float scan_time) {
  if (!std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0 ||
      !std::isfinite(grid.info.origin.position.x) ||
      !std::isfinite(grid.info.origin.position.y) || grid.info.width == 0 ||
      grid.info.height == 0 ||
      grid.data.size() !=
          static_cast<size_t>(grid.info.width) * grid.info.height ||
      !std::isfinite(sensor_x) || !std::isfinite(sensor_y) ||
      !std::isfinite(sensor_yaw) || scan_frame.empty() ||
      !std::isfinite(range_min) || !std::isfinite(range_max) ||
      !std::isfinite(angle_increment) || !(range_min > 0.0F) ||
      !(range_max > range_min) || angle_increment < 0.002F) {
    throw std::invalid_argument("ROGMap 扫描投影参数或栅格无效");
  }

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = grid.header.stamp;
  scan.header.frame_id = scan_frame;
  constexpr double kPi = 3.14159265358979323846;
  scan.angle_min = -static_cast<float>(kPi);
  scan.angle_increment = angle_increment;
  const size_t beam_count =
      static_cast<size_t>(std::floor(2.0 * kPi / angle_increment)) + 1;
  scan.angle_max = scan.angle_min +
                   static_cast<float>(beam_count - 1) * angle_increment;
  scan.time_increment = 0.0F; // 栅格是同一时刻的快照，不伪造逐束采集时间。
  scan.scan_time = scan_time;
  scan.range_min = range_min;
  scan.range_max = range_max;
  scan.ranges.assign(beam_count, std::numeric_limits<float>::infinity());

  const double step = static_cast<double>(grid.info.resolution) * 0.5;
  for (size_t beam = 0; beam < beam_count; ++beam) {
    const double angle = sensor_yaw + scan.angle_min +
                         static_cast<double>(beam) * angle_increment;
    const double dx = std::cos(angle), dy = std::sin(angle);
    for (double range = range_min; range <= range_max; range += step) {
      const double x = sensor_x + range * dx;
      const double y = sensor_y + range * dy;
      const int ix = static_cast<int>(std::floor(
          (x - grid.info.origin.position.x) / grid.info.resolution));
      const int iy = static_cast<int>(std::floor(
          (y - grid.info.origin.position.y) / grid.info.resolution));
      if (ix < 0 || iy < 0 || ix >= static_cast<int>(grid.info.width) ||
          iy >= static_cast<int>(grid.info.height)) {
        scan.ranges[beam] = std::numeric_limits<float>::quiet_NaN();
        break;
      }
      const int8_t cell =
          grid.data[static_cast<size_t>(iy) * grid.info.width + ix];
      if (cell < 0) {
        // NaN 表示没有可靠测量；inf 会让 SLAM 错误地清空未知区域。
        scan.ranges[beam] = std::numeric_limits<float>::quiet_NaN();
        break;
      }
      if (cell >= 100) {
        scan.ranges[beam] = static_cast<float>(range);
        break;
      }
    }
  }
  return scan;
}

} // namespace sentry_terrain_bridge
