#include <gtest/gtest.h>
#include <sentry_terrain_bridge/hollow_structure.hpp>

TEST(HollowStructure, PublishesOnlyClassifiedHollowColumnsWithoutClearance) {
  rog_map::ProjectionLayer layer;
  rog_map::ProjectionLayerConfig config;
  config.mask_filter_en = false;
  config.hysteresis_en = false;
  config.surface_height_delta_max = .25;
  config.wall_height_delta_min = .50;
  config.tunnel_height_delta_min = .26;
  config.tunnel_height_delta_max = .60;
  layer.updateFull(5, 1, .05, Eigen::Vector2d::Zero(), 10., config,
      [](int x, int) {
        rog_map::ColumnStats stats;
        if (x == 4) { return stats; }  // Unknown.
        stats.observed_count = 3;
        if (x == 3) { return stats; }  // Observed empty.
        stats.occupied_count = x == 2 ? 13 : (x == 1 ? 1 : 2);
        stats.occupied_z_index_min = 0;
        stats.occupied_z_index_max = x == 2 ? 12 : (x == 1 ? 0 : 8);
        stats.occupied_z_min_abs = -.2;
        stats.occupied_z_max_abs = -.2 + stats.occupied_z_index_max * .05;
        return stats;  // Hollow, thin surface, solid wall respectively.
      });
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 10;
  auto cloud = sentry_terrain_bridge::hollowStructureCloud(layer, "odom", stamp);
  ASSERT_EQ(cloud.width * cloud.height, 1U);
  EXPECT_EQ(cloud.header.frame_id, "odom");
  EXPECT_EQ(cloud.header.stamp.sec, 10);
  sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x"), z(cloud, "z"),
      ceiling(cloud, "ceiling_z"), resolution(cloud, "resolution");
  EXPECT_NEAR(*x, .025, 1e-6);
  EXPECT_NEAR(*z, -.2, 1e-6);
  EXPECT_NEAR(*ceiling, .2, 1e-6);
  EXPECT_NEAR(*resolution, .05, 1e-6);
}
