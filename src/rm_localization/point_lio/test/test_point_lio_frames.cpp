#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include "point_lio_frames.hpp"

TEST(PointLIOFrames, UsesTheSameExtrinsicsAsRegisteredClouds) {
  const double pi = std::acos(-1.);
  const Eigen::Matrix3d base_lidar_rotation =
      (Eigen::AngleAxisd(pi, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(1.3090, Eigen::Vector3d::UnitX())).toRotationMatrix();
  tf2::Transform base_lidar;
  const auto &r = base_lidar_rotation;
  base_lidar.setBasis(tf2::Matrix3x3(r(0,0),r(0,1),r(0,2),r(1,0),r(1,1),r(1,2),r(2,0),r(2,1),r(2,2)));
  base_lidar.setOrigin(tf2::Vector3(0., -.32, .176));
  const Eigen::Matrix3d imu_lidar_rotation =
      Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Vector3d imu_lidar_translation(-.011, -.02329, .04412);
  tf2::Transform identity;
  identity.setIdentity();
  const auto imu_lidar = point_lio_frames::lidarPose(identity, imu_lidar_rotation, imu_lidar_translation);
  tf2::Quaternion heading;
  heading.setRPY(.1, -.15, .4);
  const tf2::Transform expected_base(heading, tf2::Vector3(1., 2., 3.));
  const auto odom_imu = expected_base * base_lidar * imu_lidar.inverse();
  const auto odom_lidar = point_lio_frames::lidarPose(odom_imu, imu_lidar_rotation, imu_lidar_translation);
  const auto actual_base = odom_lidar * base_lidar.inverse();
  EXPECT_NEAR((actual_base.getOrigin()-expected_base.getOrigin()).length(), 0., 1e-10);
  EXPECT_NEAR(actual_base.getRotation().angleShortestPath(expected_base.getRotation()), 0., 1e-7);
  const auto scan = point_lio_frames::levelScanPose(odom_lidar, actual_base);
  EXPECT_NEAR((scan.getOrigin()-odom_lidar.getOrigin()).length(), 0., 1e-10);
  EXPECT_NEAR(scan.getBasis().getColumn(2).z(), 1., 1e-10);
  EXPECT_NEAR(tf2::getYaw(scan.getRotation()), .4, 1e-10);
}

#include "dense_terrain_registration.hpp"
#include <limits>

TEST(DenseTerrain, RegistersEachAcquisitionTimeDuringTranslationAndRotation) {
  point_lio_frames::DenseTerrainTrajectory trajectory;
  tf2::Quaternion end_rotation;
  end_rotation.setRPY(.2, -.1, .4);
  const tf2::Transform begin(tf2::Quaternion::getIdentity(), tf2::Vector3(1.,2.,3.));
  const tf2::Transform end(end_rotation, tf2::Vector3(1.1,2.2,3.3));
  ASSERT_TRUE(trajectory.append(10., begin));
  ASSERT_TRUE(trajectory.append(10.01, end));
  const tf2::Vector3 target(4.,5.,6.);
  for (int i=0; i<=10; ++i) {
    const double alpha = i / 10.;
    tf2::Transform expected(begin.getRotation().slerp(end.getRotation(), alpha),
                            begin.getOrigin().lerp(end.getOrigin(), alpha));
    const auto observed = expected.inverse() * target;
    tf2::Transform actual;
    ASSERT_TRUE(trajectory.poseAt(10. + alpha * .01, actual));
    EXPECT_NEAR((actual * observed - target).length(), 0., 1e-10);
  }
  tf2::Transform unavailable;
  EXPECT_FALSE(trajectory.poseAt(9.999, unavailable));
  EXPECT_FALSE(trajectory.poseAt(10.011, unavailable));
}

TEST(DenseTerrain, RejectsGapsAndClearsOnResetInvalidStateOrTimeRollback) {
  point_lio_frames::DenseTerrainTrajectory trajectory;
  tf2::Transform pose; pose.setIdentity();
  ASSERT_TRUE(trajectory.append(1., pose));
  ASSERT_TRUE(trajectory.append(1.1, pose));
  EXPECT_FALSE(trajectory.poseAt(1.05, pose));
  trajectory.clear();
  EXPECT_FALSE(trajectory.poseAt(1., pose));
  ASSERT_TRUE(trajectory.append(2., pose));
  ASSERT_TRUE(trajectory.append(1., pose));
  EXPECT_FALSE(trajectory.poseAt(2., pose));
  EXPECT_FALSE(trajectory.append(std::numeric_limits<double>::quiet_NaN(), pose));
  EXPECT_FALSE(trajectory.poseAt(1., pose));
  pose.setOrigin(tf2::Vector3(std::numeric_limits<double>::infinity(),0.,0.));
  EXPECT_FALSE(trajectory.append(3., pose));
}
