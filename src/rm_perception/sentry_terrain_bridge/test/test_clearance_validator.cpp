#include <gtest/gtest.h>
#include <sentry_terrain_bridge/clearance_validator.hpp>
#include <sentry_terrain_bridge/support_surface.hpp>
#include <sentry_terrain_bridge/supported_clearance.hpp>

using namespace sentry_terrain_bridge;
namespace {
ClearanceConfig config() {
  ClearanceConfig c;
  c.robot_width = c.robot_length = 0.45; c.robot_height = 0.32;
  return c;
}
PassageProbe probe() { return {0.025, 0.025, 0.025, 0.0, 1.0}; }
ObservedVoxel clear(int, int, int z) {
  return {z == 0 ? ObservedState::OCCUPIED : ObservedState::FREE, 10.0};
}
}
TEST(Clearance, FullyObservedStraightVolumePasses) {
  const auto r = validateClearance(config(), probe(), 0.05, 10.1, clear);
  EXPECT_TRUE(r.geometry_pass); EXPECT_GT(r.checked, 100U);
  EXPECT_DOUBLE_EQ(r.oldest_body_observation, 10.0);
  EXPECT_DOUBLE_EQ(r.oldest_support_observation, 10.0);
  EXPECT_DOUBLE_EQ(r.newest_support_observation, 10.0);
  EXPECT_EQ(r.recent_body_voxels, r.body_voxels);
  EXPECT_EQ(r.entry_edge_voxels, r.recent_entry_edge_voxels);
  size_t support_total = 0, support_recent = 0;
  for (size_t i = 0; i < r.support_by_slice.size(); ++i) {
    support_total += r.support_by_slice[i];
    support_recent += r.recent_support_by_slice[i];
  }
  EXPECT_EQ(support_total, r.footprint_cells);
  EXPECT_EQ(support_recent, r.footprint_cells);
  size_t total = 0, recent = 0;
  for (size_t i = 0; i < r.body_by_slice.size(); ++i) {
    total += r.body_by_slice[i]; recent += r.recent_body_by_slice[i];
  }
  EXPECT_EQ(total, r.body_voxels); EXPECT_EQ(recent, r.recent_body_voxels);
}
TEST(SupportedClearance, FollowsMeasuredRampInsteadOfCountingFloorAsBody) {
  auto p=probe();p.ground_z=-.23;
  auto ramp=[](int x,int,int z) {
    const int ground=static_cast<int>(std::floor(.2*(x+.5)));
    return ObservedVoxel{z<=ground?ObservedState::OCCUPIED:ObservedState::FREE,10.};
  };
  EXPECT_GT(validateClearance(config(),p,.05,10.1,ramp).blocked,0U);
  const auto result=validateSupportedClearance(config(),p,.05,10.1,ramp,.35);
  EXPECT_TRUE(result.clearance.geometry_pass) << result.model_reason << ' ' << result.clearance.reason;
  EXPECT_NEAR(result.slope,.2,.03);
}
TEST(SupportedClearance, DoesNotFillUnobservedSupportOrClearRoof) {
  auto ramp=[](int x,int,int z) {
    const int ground=static_cast<int>(std::floor(.2*(x+.5)));
    if(x==2)return ObservedVoxel{};
    return ObservedVoxel{z<=ground?ObservedState::OCCUPIED:ObservedState::FREE,10.};
  };
  auto missing=validateSupportedClearance(config(),probe(),.05,10.1,ramp,.35);
  EXPECT_FALSE(missing.clearance.geometry_pass);
  EXPECT_GT(missing.clearance.support_missing_cells.size(),0U);
  auto roof=validateSupportedClearance(config(),probe(),.05,10.1,
    [](int x,int,int z) {
      const int ground=static_cast<int>(std::floor(.2*(x+.5)));
      return ObservedVoxel{z<=ground || z>=ground+5?ObservedState::OCCUPIED:ObservedState::FREE,10.};
    },.35);
  EXPECT_FALSE(roof.clearance.geometry_pass);
  EXPECT_GT(roof.clearance.blocked,0U);
}
TEST(SupportedClearance, ContinuousRampToLevelUsesTwoSegments) {
  const auto result=validateSupportedClearance(config(),probe(),.05,10.1,
    [](int x,int,int z) {
      const int floor=static_cast<int>(std::floor(.3*std::min(0.0,(x+.5)*.05)/.05));
      return ObservedVoxel{z<=floor?ObservedState::OCCUPIED:ObservedState::FREE,10.};
    },.35);
  EXPECT_TRUE(result.segmented);
  EXPECT_TRUE(result.clearance.geometry_pass) << result.model_reason << ' ' << result.clearance.reason;
  EXPECT_NEAR(result.slope_after,0.0,.08);
}
TEST(SupportedClearance, TransitionDoesNotFillGapOrEraseRaisedObject) {
  for(bool gap:{false,true}) {
    const auto result=validateSupportedClearance(config(),probe(),.05,10.1,
      [gap](int x,int y,int z) {
        if(gap && x==2)return ObservedVoxel{};
        const int floor=static_cast<int>(std::floor(.3*std::min(0.0,(x+.5)*.05)/.05));
        const bool box=!gap && x>=1 && x<=3 && y>=-2 && y<=2 && z<=4;
        return ObservedVoxel{z<=floor || box?ObservedState::OCCUPIED:ObservedState::FREE,10.};
      },.35);
    EXPECT_FALSE(result.clearance.geometry_pass);
    if(gap)EXPECT_GT(result.clearance.support_missing_cells.size(),0U);
    else EXPECT_GT(result.clearance.blocked,0U);
  }
}
TEST(SupportedClearance, RearFitStripOutlierIsNotACollisionInsideTheSweep) {
  auto p=probe();p.travel_length=.2;
  auto r=validateSupportedClearance(config(),p,.05,10.1,
    [](int x,int y,int z) {
      const bool rear_object=x==-10 && y==0 && z==4;
      return ObservedVoxel{z<=0 || rear_object?ObservedState::OCCUPIED:ObservedState::FREE,10.};
    },.35);
  EXPECT_TRUE(r.clearance.geometry_pass) << r.model_reason;
  EXPECT_GT(r.approach_max_residual,.1);
  EXPECT_LT(r.max_residual,.001);
}
TEST(SupportedClearance, RejectsStepBoxSteepAndStale) {
  for(int mode=0;mode<4;++mode) {
    auto result=validateSupportedClearance(config(),probe(),.05,10.1,
      [mode](int x,int y,int z) {
        int level=0;
        if(mode==0 && x>0)level=4;
        if(mode==1 && x>=0 && x<=3 && y>=-2 && y<=2)level=4;
        if(mode==2)level=static_cast<int>(std::floor(.6*(x+.5)));
        return ObservedVoxel{z<=level?ObservedState::OCCUPIED:ObservedState::FREE,mode==3?1.:10.};
      },.35);
    EXPECT_FALSE(result.clearance.geometry_pass) << mode;
  }
}
TEST(SupportSurface, FindsObservedPlaneWithoutChangingProbe) {
  auto p=probe();p.ground_z=-.23;
  const auto r=estimateSupportSurface(config(),p,.05,10.1,clear);
  ASSERT_TRUE(r.complete);
  EXPECT_NEAR(r.ground_z,.025,1e-9);
  EXPECT_DOUBLE_EQ(p.ground_z,-.23);
}
TEST(SupportSurface, RejectsUnknownApproachAndMultipleDecks) {
  const auto unknown=estimateSupportSurface(config(),probe(),.05,10.1,
    [](int,int,int){return ObservedVoxel{};});
  EXPECT_FALSE(unknown.complete);
  EXPECT_EQ(unknown.reason,"NO_OBSERVED_APPROACH_SUPPORT");
  const auto decks=estimateSupportSurface(config(),probe(),.05,10.1,
    [](int,int,int z){return ObservedVoxel{
      (z==0||z==5)?ObservedState::OCCUPIED:ObservedState::FREE,10.};});
  EXPECT_FALSE(decks.complete);
  EXPECT_GT(decks.ambiguous,0U);
}
TEST(SupportSurface, RejectsStepInsteadOfTreatingBoxTopAsFloor) {
  const auto r=estimateSupportSurface(config(),probe(),.05,10.1,
    [](int x,int,int z){return ObservedVoxel{
      z==(x<0?0:3)?ObservedState::OCCUPIED:ObservedState::FREE,10.};});
  EXPECT_FALSE(r.complete);
  EXPECT_EQ(r.reason,"SUPPORT_STEP_OR_SLOPE");
}
TEST(SupportSurface, KeepsSlopeAndUnobservedSeamVisibleWithoutApprovingIt) {
  const auto r=estimateSupportSurface(config(),probe(),.05,10.1,
    [](int x,int,int z){
      if(x>=2 && x<=4)return ObservedVoxel{};
      const int level=x<0?0:(x<4?1:2);
      return ObservedVoxel{z==level?ObservedState::OCCUPIED:ObservedState::FREE,10.};
    });
  EXPECT_FALSE(r.complete);
  EXPECT_GT(r.rows.size(),2U);
  EXPECT_GT(r.missing,0U);
  EXPECT_EQ(r.unobserved_columns.size(),r.missing);
  EXPECT_GT(r.max_observed_row_delta,0.0);
  EXPECT_EQ(r.max_row_spread,0.0);
}
TEST(Clearance, BodyFreshnessIsSeparateFromOlderStructuralGround) {
  auto c = config(); c.max_observation_age = 20.0;
  const auto r = validateClearance(c, probe(), 0.05, 10.1,
    [](int, int, int z) {
      return ObservedVoxel{z == 0 ? ObservedState::OCCUPIED : ObservedState::FREE,
                           z == 0 ? 4.0 : 10.0};
    });
  ASSERT_TRUE(r.geometry_pass);
  EXPECT_DOUBLE_EQ(r.oldest_observation, 4.0);
  EXPECT_DOUBLE_EQ(r.oldest_body_observation, 10.0);
  EXPECT_DOUBLE_EQ(r.oldest_support_observation, 4.0);
  EXPECT_DOUBLE_EQ(r.newest_support_observation, 4.0);
  EXPECT_EQ(r.recent_support_by_slice[3], 0U);
  EXPECT_EQ(r.entry_edge_voxels, r.recent_body_by_slice[6] +
            r.recent_body_by_slice[7] + r.support_by_slice[3]);
  EXPECT_LT(r.recent_entry_edge_voxels, r.entry_edge_voxels);
  EXPECT_EQ(r.recent_body_voxels, r.body_voxels);
}
TEST(Clearance, ReportsNewestGroundObservationSeparately) {
  auto c = config(); c.max_observation_age = 20.0;
  const auto r = validateClearance(c, probe(), 0.05, 10.1,
    [](int x, int, int z) {
      return ObservedVoxel{z == 0 ? ObservedState::OCCUPIED : ObservedState::FREE,
                           z == 0 ? (x < 0 ? 4.0 : 9.8) : 10.0};
    });
  ASSERT_TRUE(r.geometry_pass);
  EXPECT_DOUBLE_EQ(r.oldest_support_observation, 4.0);
  EXPECT_DOUBLE_EQ(r.newest_support_observation, 9.8);
  EXPECT_DOUBLE_EQ(r.oldest_body_observation, 10.0);
}
TEST(Clearance, RecentBodyCoverageCountsNewRaysNotHistoricalFreeSpace) {
  auto c = config(); c.max_observation_age = 20.0;
  const auto r = validateClearance(c, probe(), 0.05, 10.1,
    [](int x, int, int z) {
      return ObservedVoxel{z == 0 ? ObservedState::OCCUPIED : ObservedState::FREE,
                           x < 0 ? 4.0 : 10.0};
    });
  ASSERT_TRUE(r.geometry_pass);
  EXPECT_GT(r.recent_body_voxels, 0U);
  EXPECT_LT(r.recent_body_voxels, r.body_voxels);
}
TEST(Clearance, SeparatesUnknownBodyFromSupportEvidence) {
  const auto body = validateClearance(config(), probe(), .05, 10.1,
    [](int x, int y, int z) {
      return z > 0 ? ObservedVoxel{} : clear(x,y,z);
    });
  EXPECT_EQ(body.unknown, body.body_unknown);
  EXPECT_EQ(body.body_unknown_cells.size(), body.body_unknown);
  EXPECT_TRUE(body.support_missing_cells.empty());
  const auto support = validateClearance(config(), probe(), .05, 10.1,
    [](int x, int y, int z) { return z == 0 ? ObservedVoxel{} : clear(x,y,z); });
  EXPECT_EQ(support.body_unknown, 0U);
  EXPECT_EQ(support.support_unknown, support.footprint_cells);
  EXPECT_EQ(support.support_missing_cells.size(), support.footprint_cells);
  EXPECT_FALSE(support.geometry_pass);
}
TEST(Clearance, SupportMissingReasonsAreDisjointAndMarkersClearOnSuccess) {
  for (auto state : {ObservedState::FREE, ObservedState::OUTSIDE, ObservedState::OCCUPIED}) {
    const auto r = validateClearance(config(), probe(), .05, 10.1,
      [state](int x, int y, int z) {
        return z == 0 ? ObservedVoxel{state, 1.0} : clear(x,y,z);
      });
    EXPECT_EQ(r.support_missing_cells.size(), r.footprint_cells);
    EXPECT_EQ(r.support_free + r.support_outside + r.support_stale, r.footprint_cells);
    EXPECT_FALSE(r.geometry_pass);
  }
  const auto r = validateClearance(config(), probe(), .05, 10.1, clear);
  EXPECT_TRUE(r.body_unknown_cells.empty());
  EXPECT_TRUE(r.support_missing_cells.empty());
}
TEST(Clearance, UnknownGapAndSparseWallFail) {
  auto query = [](int x, int y, int z) {
    if (x == 0 && z > 0) { return ObservedVoxel{y == 0 ? ObservedState::OCCUPIED : ObservedState::UNKNOWN, 10.0}; }
    return clear(x, y, z);
  };
  auto r = validateClearance(config(), probe(), 0.05, 10.1, query);
  EXPECT_FALSE(r.geometry_pass); EXPECT_GT(r.unknown, 0U); EXPECT_GT(r.blocked, 0U);
}
TEST(Clearance, LowBeamFails) {
  auto r = validateClearance(config(), probe(), 0.05, 10.1, [](int x, int y, int z) {
    if (x == 0 && z == 6) { return ObservedVoxel{ObservedState::OCCUPIED, 10.0}; }
    return clear(x, y, z);
  });
  EXPECT_EQ(r.reason, "BODY_VOLUME_OCCUPIED");
  ASSERT_EQ(r.body_blocked_cells.size(), r.blocked);
  ASSERT_GT(r.blocked, 0U);
  for (const auto & cell : r.body_blocked_cells) {
    EXPECT_NEAR(cell.x, 0.025, 1e-9);
    EXPECT_NEAR(cell.z, 0.325, 1e-9);
  }
}
TEST(Clearance, NarrowDoorFailsEvenWithFreeCenterline) {
  auto r = validateClearance(config(), probe(), 0.05, 10.1, [](int x, int y, int z) {
    if (x == 0 && std::abs(y) >= 3 && z > 0) { return ObservedVoxel{ObservedState::OCCUPIED, 10.0}; }
    return clear(x, y, z);
  });
  EXPECT_FALSE(r.geometry_pass); EXPECT_GT(r.blocked, 0U);
}
TEST(Clearance, NoFloorSupportFails) {
  auto r = validateClearance(config(), probe(), 0.05, 10.1, [](int, int, int) {
    return ObservedVoxel{ObservedState::FREE, 10.0};
  });
  EXPECT_EQ(r.reason, "NO_GROUND_SUPPORT");
}
TEST(Clearance, UnknownAndOutsideFail) {
  for (auto state : {ObservedState::UNKNOWN, ObservedState::OUTSIDE}) {
    EXPECT_FALSE(validateClearance(config(), probe(), 0.05, 10.1,
      [state](int, int, int) { return ObservedVoxel{state, 10.0}; }).geometry_pass);
  }
}
TEST(Clearance, OldFutureAndDecayOnlyFreeAreNotEvidence) {
  for (double stamp : {0.0, 8.0, 11.0, std::numeric_limits<double>::quiet_NaN()}) {
    auto r = validateClearance(config(), probe(), 0.05, 10.1,
      [stamp](int x, int y, int z) { auto v = clear(x,y,z); v.direct_stamp = stamp; return v; });
    EXPECT_FALSE(r.geometry_pass); EXPECT_GT(r.stale, 0U);
  }
}
TEST(Clearance, YawAndFullVehicleLengthAreIncluded) {
  auto p = probe(); p.yaw = 1.5707963267948966;
  auto r = validateClearance(config(), p, 0.05, 10.1, [](int x, int y, int z) {
    // 车中心行程只有 1 m，但车身扫掠到 y=0.625，不能漏检车头车尾。
    if (y == 12 && z == 5) { return ObservedVoxel{ObservedState::OCCUPIED, 10.0}; }
    return clear(x,y,z);
  });
  EXPECT_FALSE(r.geometry_pass);
}
TEST(Clearance, InvalidConfigAndHugeRequestsFailWithoutQueries) {
  int calls = 0;
  auto query = [&](int x, int y, int z) { ++calls; return clear(x,y,z); };
  auto c = config(); c.robot_width = 0;
  EXPECT_FALSE(validateClearance(c, probe(), 0.05, 10.1, query).geometry_pass);
  auto p = probe(); p.travel_length = 1e100;
  EXPECT_FALSE(validateClearance(config(), p, 0.05, 10.1, query).geometry_pass);
  EXPECT_EQ(calls, 0);
}
TEST(Clearance, ConfirmationNeedsNewDirectObservationsAndRevokes) {
  ClearanceConfirmation tracker;
  auto c = config();
  auto r = validateClearance(c, probe(), 0.05, 10.1, clear);
  EXPECT_EQ(tracker.update(r, 1, 10.1, c), 1);
  EXPECT_EQ(tracker.update(r, 1, 10.2, c), 1);
  r.newest_observation = 10.2; EXPECT_EQ(tracker.update(r, 1, 10.3, c), 2);
  r.newest_observation = 10.3; EXPECT_EQ(tracker.update(r, 1, 10.4, c), 3);
  r.geometry_pass = false; EXPECT_EQ(tracker.update(r, 1, 10.5, c), 0);
  r.geometry_pass = true; r.newest_observation = 10.5;
  EXPECT_EQ(tracker.update(r, 1, 10.6, c), 1);
  EXPECT_EQ(tracker.update(r, 2, 10.7, c), 1);
  EXPECT_EQ(tracker.update(r, 2, 12.0, c), 0);
}
