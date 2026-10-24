#include <cmath>
#include <limits>

#include <gtest/gtest.h>
#include <sentry_terrain_bridge/grid_scan.hpp>

namespace {
nav_msgs::msg::OccupancyGrid makeGrid() {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.resolution = 0.1F;
  grid.info.width = 41;
  grid.info.height = 41;
  grid.info.origin.position.x = -2.05;
  grid.info.origin.position.y = -2.05;
  grid.info.origin.orientation.w = 1.0;
  grid.data.assign(41 * 41, 0);
  return grid;
}

void setCell(nav_msgs::msg::OccupancyGrid &grid, double x, double y,
             int8_t value) {
  const auto ix = static_cast<size_t>(std::floor(
      (x - grid.info.origin.position.x) / grid.info.resolution));
  const auto iy = static_cast<size_t>(std::floor(
      (y - grid.info.origin.position.y) / grid.info.resolution));
  grid.data[iy * grid.info.width + ix] = value;
}
} // namespace

TEST(GridScan, WallReturnsFiniteRangeButOpeningStaysOpen) {
  auto grid = makeGrid();
  // 正前方开口，左右两侧有墙；不能把开口自动补成障碍。
  setCell(grid, 1.0, 0.36, 100);
  setCell(grid, 1.0, -0.36, 100);
  auto scan = sentry_terrain_bridge::projectGridToScan(
      grid, 0.0, 0.0, 0.0, "livox_frame", 0.1F, 1.8F,
      static_cast<float>(3.14159265358979323846 / 18.0), 0.1F);
  EXPECT_EQ(scan.header.frame_id, "livox_frame");
  EXPECT_TRUE(std::isinf(scan.ranges[18]));
  EXPECT_TRUE(std::isfinite(scan.ranges[20]));
  EXPECT_TRUE(std::isfinite(scan.ranges[16]));
}

TEST(GridScan, UnknownDoesNotBecomeWall) {
  auto grid = makeGrid();
  setCell(grid, 0.5, 0.0, -1);
  setCell(grid, 1.0, 0.0, 100);
  auto scan = sentry_terrain_bridge::projectGridToScan(
      grid, 0.0, 0.0, 0.0, "livox_frame", 0.1F, 1.8F,
      static_cast<float>(3.14159265358979323846 / 18.0), 0.1F);
  EXPECT_TRUE(std::isnan(scan.ranges[18]));
}

TEST(GridScan, EmptyAndUnobservedScansCannotInitializeMapping) {
  sensor_msgs::msg::LaserScan scan;
  scan.range_min = 0.48F;
  scan.range_max = 4.5F;
  EXPECT_FALSE(sentry_terrain_bridge::hasUsableMappingReturn(scan));
  scan.ranges = {std::numeric_limits<float>::quiet_NaN(),
                 std::numeric_limits<float>::infinity(),
                 -std::numeric_limits<float>::infinity(),
                 0.0F, 0.48F, 4.5F, 5.0F};
  EXPECT_FALSE(sentry_terrain_bridge::hasUsableMappingReturn(scan));
  scan.ranges.push_back(1.2F);
  EXPECT_TRUE(sentry_terrain_bridge::hasUsableMappingReturn(scan));
}

TEST(GridScan, WaitForWallWhileContinuingToUpdateTerrain) {
  auto grid = makeGrid();
  auto scan = sentry_terrain_bridge::projectGridToScan(
      grid, 0.0, 0.0, 0.0, "livox_frame", 0.1F, 1.8F,
      static_cast<float>(3.14159265358979323846 / 18.0), 0.1F);
  EXPECT_FALSE(sentry_terrain_bridge::hasUsableMappingReturn(scan));
  setCell(grid, 1.0, 0.0, 100);
  scan = sentry_terrain_bridge::projectGridToScan(
      grid, 0.0, 0.0, 0.0, "livox_frame", 0.1F, 1.8F,
      static_cast<float>(3.14159265358979323846 / 18.0), 0.1F);
  EXPECT_TRUE(sentry_terrain_bridge::hasUsableMappingReturn(scan));
}
