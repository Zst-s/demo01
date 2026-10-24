#pragma once
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include <fcntl.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>

namespace point_lio_frames {
// Publish a complete new file atomically; never replace an existing map.
template<class Point>
void saveMapSnapshot(const pcl::PointCloud<Point> &world,
    const geometry_msgs::msg::TransformStamped &transform,
    const std::string &destination) {
  if (world.empty() || destination.empty() || destination.front() != '/' ||
      destination.size() < 4 || destination.substr(destination.size()-4) != ".pcd") {
    throw std::invalid_argument("Snapshot requires nonempty cloud and absolute .pcd path");
  }
  const auto &t = transform.transform.translation;
  const auto &q = transform.transform.rotation;
  Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
  if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-12 ||
      !std::isfinite(t.x) || !std::isfinite(t.y) || !std::isfinite(t.z)) {
    throw std::invalid_argument("Invalid snapshot transform");
  }
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3,3>(0,0) = rotation.normalized().toRotationMatrix().cast<float>();
  pose.block<3,1>(0,3) = Eigen::Vector3f(t.x,t.y,t.z);
  pcl::PointCloud<Point> mapped;
  pcl::transformPointCloud(world, mapped, pose);
  std::string pattern = destination + ".tmp-XXXXXX";
  std::vector<char> name(pattern.begin(),pattern.end());name.push_back('\0');
  const int fd = ::mkstemp(name.data());
  if (fd < 0) throw std::runtime_error(std::string("Cannot create PCD temporary file: ")+std::strerror(errno));
  ::close(fd);
  try {
    pcl::PCDWriter writer;
    if (writer.writeBinary(name.data(), mapped) < 0) throw std::runtime_error("PCD writer failed");
    const int sync_fd = ::open(name.data(), O_RDONLY);
    if (sync_fd < 0) throw std::runtime_error("Cannot reopen saved PCD");
    const int sync_result = ::fsync(sync_fd);::close(sync_fd);
    if (sync_result != 0) throw std::runtime_error("Cannot sync saved PCD");
    if (::link(name.data(),destination.c_str()) != 0) {
      throw std::runtime_error(std::string("Cannot publish PCD (existing maps are preserved): ")+std::strerror(errno));
    }
  } catch (...) {::unlink(name.data());throw;}
  ::unlink(name.data());
}
} // namespace point_lio_frames
