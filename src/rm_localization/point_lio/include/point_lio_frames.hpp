#pragma once

#include <cmath>
#include <Eigen/Core>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>

namespace point_lio_frames {
inline tf2::Transform lidarPose(const tf2::Transform &odom_imu,
    const Eigen::Matrix3d &imu_lidar_rotation, const Eigen::Vector3d &imu_lidar_translation) {
  tf2::Transform imu_lidar;
  const auto &r = imu_lidar_rotation;
  imu_lidar.setBasis(tf2::Matrix3x3(r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2),
                                  r(2,0), r(2,1), r(2,2)));
  imu_lidar.setOrigin(tf2::Vector3(imu_lidar_translation.x(), imu_lidar_translation.y(),
                                 imu_lidar_translation.z()));
  return odom_imu * imu_lidar;
}

// Same lidar origin; horizontal axes follow base yaw in gravity-aligned odom.
inline tf2::Transform levelScanPose(const tf2::Transform &odom_lidar,
                                    const tf2::Transform &odom_base) {
  tf2::Quaternion yaw;
  yaw.setRPY(0., 0., tf2::getYaw(odom_base.getRotation()));
  return tf2::Transform(yaw, odom_lidar.getOrigin());
}
}  // namespace point_lio_frames
