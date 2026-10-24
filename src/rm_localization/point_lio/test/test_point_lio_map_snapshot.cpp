#include <gtest/gtest.h>
#include <pcl/point_types.h>
#include <fstream>
#include "point_lio_map_snapshot.hpp"

TEST(PointLIOMapSnapshot, AppliesFullMapTransformAndPreservesExistingMap) {
  char directory[] = "/tmp/pointlio-snapshot-XXXXXX";
  ASSERT_NE(::mkdtemp(directory), nullptr);
  const std::string file = std::string(directory) + "/new.pcd";
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI point;point.x=1;point.y=2;point.z=3;point.intensity=17;
  cloud.push_back(point);
  geometry_msgs::msg::TransformStamped tf;
  tf.transform.translation.x=4;tf.transform.translation.y=-2;tf.transform.translation.z=.5;
  Eigen::Quaterniond q(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(-.2,Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(.3,Eigen::Vector3d::UnitX()));
  tf.transform.rotation.x=q.x();tf.transform.rotation.y=q.y();
  tf.transform.rotation.z=q.z();tf.transform.rotation.w=q.w();
  ASSERT_NO_THROW(point_lio_frames::saveMapSnapshot(cloud,tf,file));
  pcl::PointCloud<pcl::PointXYZI> read;
  ASSERT_GE(pcl::io::loadPCDFile(file,read),0);ASSERT_EQ(read.size(),1u);
  const Eigen::Vector3d expected=q*Eigen::Vector3d(1,2,3)+Eigen::Vector3d(4,-2,.5);
  EXPECT_NEAR(read[0].x,expected.x(),1e-5);EXPECT_NEAR(read[0].y,expected.y(),1e-5);
  EXPECT_NEAR(read[0].z,expected.z(),1e-5);EXPECT_FLOAT_EQ(read[0].intensity,17);
  EXPECT_FLOAT_EQ(cloud[0].x,1); // exporting must not alter live localization points
  cloud[0].x=100;
  EXPECT_THROW(point_lio_frames::saveMapSnapshot(cloud,tf,file),std::runtime_error);
  ASSERT_GE(pcl::io::loadPCDFile(file,read),0);EXPECT_NEAR(read[0].x,expected.x(),1e-5);
  ::unlink(file.c_str());::rmdir(directory);
}

TEST(PointLIOMapSnapshot, RejectsMissingCloudInvalidTransformAndUnwritableDestination) {
  pcl::PointCloud<pcl::PointXYZI> cloud;
  geometry_msgs::msg::TransformStamped tf;tf.transform.rotation.w=1;
  EXPECT_THROW(point_lio_frames::saveMapSnapshot(cloud,tf,"/tmp/unused-map.pcd"),std::invalid_argument);
  pcl::PointXYZI point;point.x=1;point.y=2;point.z=3;cloud.push_back(point);
  tf.transform.rotation.w=0;
  EXPECT_THROW(point_lio_frames::saveMapSnapshot(cloud,tf,"/tmp/unused-map.pcd"),std::invalid_argument);
  tf.transform.rotation.w=1;
  EXPECT_THROW(point_lio_frames::saveMapSnapshot(cloud,tf,"/proc/no-map-directory/new.pcd"),std::runtime_error);
}
