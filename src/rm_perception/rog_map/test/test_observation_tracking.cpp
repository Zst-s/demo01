#include <gtest/gtest.h>
#include <rog_map/prob_map.h>

namespace {
class ProbeMap : public rog_map::ProbMap {
public:
  explicit ProbeMap(const rog_map::Config & config) { cfg_ = config; initProbMap(); }
  void hit(const super_utils::Vec3i & id, double stamp) {
    super_utils::Vec3f pos; globalIndexToPos(id, pos);
    setUpdateTime(stamp); hitPointUpdate(pos, getHashIndexFromGlobalIndex(id), 10);
  }
  void miss(const super_utils::Vec3i & id, double stamp) {
    super_utils::Vec3f pos; globalIndexToPos(id, pos);
    setUpdateTime(stamp); missPointUpdate(pos, getHashIndexFromGlobalIndex(id), 100);
  }
  void resetOne(const super_utils::Vec3i & id) { resetCell(getHashIndexFromGlobalIndex(id)); }
  void resetAll() { resetLocalMap(); }
  super_utils::Vec3i virtualFloor() const { return {0, 0, sc_.virtual_ground_height_id_g}; }
};
}  // namespace

TEST(ObservationTracking, OnlyDirectEvidenceSurvivesAndResetsRevokeIt)
{
  rclcpp::init(0, nullptr);
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    rclcpp::Parameter("rog_map.map_size", std::vector<double>{4.0, 4.0, 4.0}),
    rclcpp::Parameter("rog_map.raycasting.p_miss", 0.45),
    rclcpp::Parameter("rog_map.decay.keep_time", 1.0),
    rclcpp::Parameter("rog_map.decay.clear_time", 2.0),
    rclcpp::Parameter("rog_map.performance.enable", false)});
  auto node = std::make_shared<rclcpp::Node>("observation_tracking_test", options);
  rog_map::Config config;
  config.loadFromRosNode(node, "rog_map");
  ProbeMap map(config);
  const super_utils::Vec3i id(2, 2, 2);
  using Type = super_utils::GridType;
  map.hit(id, 1000.0);
  EXPECT_EQ(map.cellDirectObservationTime(id), 0.0);  // opt-in, no retrospective trust
  map.enableObservationTracking();
  EXPECT_EQ(map.cellDirectObservationTime(id), 0.0);
  map.hit(id, 1791970000.125);
  EXPECT_DOUBLE_EQ(map.cellDirectObservationTime(id), 1791970000.125);
  EXPECT_EQ(map.rawObservedType(id), Type::OCCUPIED);
  map.miss(id, 1791970000.375);
  EXPECT_DOUBLE_EQ(map.cellDirectObservationTime(id), 1791970000.375);
  EXPECT_EQ(map.rawObservedType(id), Type::KNOWN_FREE);
  map.hit(id, 1791970001.125);
  ASSERT_EQ(map.rawObservedType(id), Type::OCCUPIED);
  ASSERT_TRUE(map.applyDecay(1791970004.0));
  EXPECT_EQ(map.rawObservedType(id), Type::KNOWN_FREE);
  EXPECT_EQ(map.cellDirectObservationTime(id), 0.0);  // decay is NOT a free-space observation
  map.hit(id, 1791970005.125);
  map.resetOne(id);
  EXPECT_EQ(map.cellDirectObservationTime(id), 0.0);
  EXPECT_EQ(map.rawObservedType(id), Type::UNKNOWN);
  map.hit(id, 1791970006.125);
  map.resetAll();
  EXPECT_EQ(map.cellDirectObservationTime(id), 0.0);
  EXPECT_EQ(map.rawObservedType(id), Type::UNKNOWN);
  EXPECT_EQ(map.rawObservedType(map.virtualFloor()), Type::UNKNOWN);
  EXPECT_EQ(map.rawObservedType(super_utils::Vec3i(10000, 0, 0)), Type::OUT_OF_MAP);
  node.reset();
  rclcpp::shutdown();
}
