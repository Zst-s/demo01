#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace sentry_terrain_bridge {
// ROGMap 是 Nav2 风格代价值：255 未知，254 障碍，0 自由；不是 OccupancyGrid
// 百分比。
inline int8_t toOccupancy(uint8_t cost) {
  if (cost == 255) {
    return -1;
  }
  if (cost >= 254) {
    return 100;
  }
  if (cost == 0) {
    return 0;
  }
  return static_cast<int8_t>(
      std::clamp(static_cast<int>(std::lround(cost * 99.0 / 253.0)), 1, 99));
}

inline unsigned char toCost(int8_t occupancy) {
  if (occupancy < 0) {
    return 255;
  }
  if (occupancy >= 100) {
    return 254;
  }
  if (occupancy == 0) {
    return 0;
  }
  return static_cast<unsigned char>(std::lround(occupancy * 253.0 / 99.0));
}

// 负方向必须向下取整，否则地图左/下边界之外的点会错误落入第 0 格。
inline int cellIndex(double position, double origin, double resolution) {
  return static_cast<int>(std::floor((position - origin) / resolution));
}
} // namespace sentry_terrain_bridge
