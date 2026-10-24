#include <gtest/gtest.h>
#include <sentry_terrain_bridge/grid_contract.hpp>
using namespace sentry_terrain_bridge;

TEST(GridContract, UnknownIsNotAnObstacleOrFreeSpace) {
  EXPECT_EQ(toOccupancy(255), -1);
  EXPECT_EQ(toCost(-1), 255);
}
TEST(GridContract, LethalAndFreeArePreserved) {
  EXPECT_EQ(toOccupancy(254), 100);
  EXPECT_EQ(toOccupancy(0), 0);
  EXPECT_EQ(toCost(100), 254);
  EXPECT_EQ(toCost(0), 0);
}
TEST(GridContract, IntermediateCostsDoNotBecomeLethal) {
  for (int cost = 1; cost < 254; ++cost) {
    EXPECT_GT(toOccupancy(cost), 0);
    EXPECT_LT(toOccupancy(cost), 100);
    EXPECT_LT(toCost(toOccupancy(cost)), 254);
  }
}
TEST(GridContract, NegativeCoordinatesUseFloor) {
  EXPECT_EQ(cellIndex(-0.001, 0.0, 0.05), -1);
  EXPECT_EQ(cellIndex(0.049, 0.0, 0.05), 0);
  EXPECT_EQ(cellIndex(-4.95, -5.0, 0.1), 0);
}
