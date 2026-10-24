#include <gtest/gtest.h>

#include <rog_map/projection_layer.hpp>
#include <rog_map/projection_reason_markers.hpp>

namespace {

rog_map::ColumnStats thinOccupiedColumn(double z)
{
  rog_map::ColumnStats stats;
  stats.observed_count = 3;
  stats.occupied_count = 1;
  stats.occupied_z_index_min = 0;
  stats.occupied_z_index_max = 0;
  stats.occupied_z_min_abs = z;
  stats.occupied_z_max_abs = z;
  return stats;
}

uint8_t centerValue(rog_map::ProjectionLayerConfig config, double z)
{
  config.mask_filter_en = false;
  config.hysteresis_en = false;
  rog_map::ProjectionLayer layer;
  const auto scanner = [z](int x, int y) {
    if (x == 1 && y == 1) {
      return thinOccupiedColumn(z);
    }
    rog_map::ColumnStats free;
    free.observed_count = 2;
    return free;
  };
  layer.updateFull(3, 3, 0.05, Eigen::Vector2d::Zero(), 1.0, config, scanner);
  return layer.values().at(4);
}

}  // namespace

TEST(ProjectionLayer, ThinSurfaceRemainsPassableAtDifferentHeights)
{
  rog_map::ProjectionLayerConfig config;
  config.surface_height_delta_max = 0.25;

  EXPECT_EQ(centerValue(config, -0.25), 0U);
  EXPECT_EQ(centerValue(config, 0.10), 0U);
  EXPECT_EQ(centerValue(config, 0.35), 0U);
}

TEST(ProjectionLayer, ReasonMarkersAreReadOnlyAndDeleteEmptyGroups)
{
  rog_map::ProjectionLayerConfig config;
  config.mask_filter_en = false;
  config.hysteresis_en = false;
  rog_map::ProjectionLayer layer;
  layer.updateFull(3, 1, 0.1, Eigen::Vector2d(1.0, 2.0), 1.0, config,
    [](int x, int) {
      if (x == 0) { return thinOccupiedColumn(-0.2); }
      rog_map::ColumnStats s;
      if (x == 1) { s.observed_count = 3; }
      return s;
    });
  const auto before = layer.values();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 7;
  const auto markers = rog_map::projectionReasonMarkers(layer, "odom", stamp);
  ASSERT_EQ(markers.markers.size(), 7U);
  EXPECT_EQ(markers.markers[2].points.size(), 1U);
  EXPECT_EQ(markers.markers[1].points.size(), 1U);
  EXPECT_EQ(markers.markers[0].points.size(), 1U);
  EXPECT_EQ(markers.markers[4].action, visualization_msgs::msg::Marker::DELETE);
  EXPECT_EQ(markers.markers[2].header.frame_id, "odom");
  EXPECT_EQ(markers.markers[2].header.stamp.sec, 7);
  EXPECT_NEAR(markers.markers[2].points[0].x, 1.05, 1e-9);
  EXPECT_NEAR(markers.markers[2].points[0].y, 2.05, 1e-9);
  EXPECT_EQ(layer.values(), before);
}

TEST(ProjectionBoundary, InclusiveTunnelMaximumAndNextVoxel)
{
  rog_map::ProjectionLayerConfig config;
  config.mask_filter_en = false;
  config.hysteresis_en = false;
  config.surface_height_delta_max = .25;
  config.wall_height_delta_min = .50;
  config.wall_occupancy_ratio_min = .80;
  config.tunnel_height_delta_min = .26;
  config.tunnel_height_delta_max = .60;
  config.tunnel_occupancy_ratio_max = .60;
  for (const int steps : {11, 12, 13}) {
    rog_map::ProjectionLayer layer;
    layer.updateFull(3, 3, .05, Eigen::Vector2d::Zero(), 1., config,
      [steps](int x, int y) {
        rog_map::ColumnStats stats;
        stats.observed_count = 20;
        if (x == 1 && y == 1) {
          stats.occupied_count = 3;
          stats.occupied_z_index_min = 10;
          stats.occupied_z_index_max = 10 + steps;
          stats.occupied_z_min_abs = .525;
          stats.occupied_z_max_abs = .525 + steps*.05;
        }
        return stats;
      });
    const auto expected = steps <= 12 ? rog_map::ProjectionClassReason::HOLLOW_TUNNEL :
      rog_map::ProjectionClassReason::AMBIGUOUS_OCCUPIED;
    EXPECT_EQ(layer.cells().at(4).raw_reason, expected) << "span_steps=" << steps;
  }
}
