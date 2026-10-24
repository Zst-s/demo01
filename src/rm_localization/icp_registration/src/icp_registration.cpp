#include "icp_registration/icp_registration.hpp"
#include <Eigen/src/Geometry/Quaternion.h>
#include <Eigen/src/Geometry/Transform.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <algorithm>
#include <pcl/filters/filter.h>
#include <geometry_msgs/msg/detail/pose_with_covariance_stamped__struct.hpp>
#include <iostream>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/qos.hpp>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/create_timer_ros.h>

namespace icp {

IcpNode::IcpNode(const rclcpp::NodeOptions &options)
    : Node("icp_registration", options), rough_iter_(10), refine_iter_(5),
      first_scan_(true) {
  is_ready_ = false;
  cloud_in_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  // this->declare_parameter("use_sim_time", false);
  double rough_leaf_size = this->declare_parameter("rough_leaf_size", 0.4);
  double refine_leaf_size = this->declare_parameter("refine_leaf_size", 0.1);
  voxel_rough_filter_.setLeafSize(rough_leaf_size, rough_leaf_size,
                                  rough_leaf_size);
  voxel_refine_filter_.setLeafSize(refine_leaf_size, refine_leaf_size,
                                   refine_leaf_size);

  pcd_path_ = this->declare_parameter("pcd_path", std::string(""));
  if (!std::filesystem::exists(pcd_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Invalid pcd path: %s", pcd_path_.c_str());
    throw std::runtime_error("Invalid pcd path");
  }
  // Read the pcd file
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (reader.read(pcd_path_, *cloud) < 0 || cloud->empty()) {
    throw std::runtime_error("Cannot read nonempty PCD map");
  }
  voxel_refine_filter_.setInputCloud(cloud);
  voxel_refine_filter_.filter(*cloud);

  // Add normal to the pointcloud
  refine_map_ = addNorm(cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterd_point_rough(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::copyPointCloud(*refine_map_, *point_rough);
  voxel_rough_filter_.setInputCloud(point_rough);
  voxel_rough_filter_.filter(*filterd_point_rough);
  rough_map_ = addNorm(filterd_point_rough);

  icp_rough_.setMaximumIterations(rough_iter_);
  icp_rough_.setInputTarget(rough_map_);

  icp_refine_.setMaximumIterations(refine_iter_);
  icp_refine_.setInputTarget(refine_map_);

  RCLCPP_INFO(this->get_logger(), "pcd point size: %ld, %ld",
              refine_map_->size(), rough_map_->size());

  // Parameters
  map_frame_id_ = this->declare_parameter("map_frame_id", std::string("map"));
  odom_frame_id_ =
      this->declare_parameter("odom_frame_id", std::string("odom"));
  range_odom_frame_id_ =
      this->declare_parameter("range_odom_frame_id", std::string("odom"));
  laser_frame_id_ =
      this->declare_parameter("laser_frame_id", std::string("laser"));
  base_frame_id_ = this->declare_parameter("base_frame_id", std::string("base_link"));
  thresh_ = this->declare_parameter("thresh", 0.15);
  xy_offset_ = this->declare_parameter("xy_offset", 0.2);
  yaw_offset_ = this->declare_parameter("yaw_offset", 30.0) * M_PI / 180.0;
  yaw_resolution_ =
      this->declare_parameter("yaw_resolution", 10.0) * M_PI / 180.0;
  if (!std::isfinite(yaw_resolution_) || yaw_resolution_ <= 0.0 ||
      !std::isfinite(yaw_offset_) || yaw_offset_ < 0.0 || yaw_offset_ > M_PI ||
      yaw_offset_ / yaw_resolution_ > 360.0 ||
      !std::isfinite(thresh_) || thresh_ <= 0.0 ||
      !std::isfinite(xy_offset_) || xy_offset_ < 0.0 ||
      odom_frame_id_ != range_odom_frame_id_) {
    throw std::invalid_argument("Invalid ICP search parameters or inconsistent odom frames");
  }
  // Bound correspondences at each voxel scale; distant guesses must fail,
  // rather than fitting unrelated points with PCL's unlimited default.
  icp_rough_.setMaxCorrespondenceDistance(2.0 * rough_leaf_size);
  icp_refine_.setMaxCorrespondenceDistance(
      std::max(2.0 * refine_leaf_size, 2.0 * std::sqrt(thresh_)));
  std::vector<double> initial_pose_vec = this->declare_parameter(
      "initial_pose", std::vector<double>{0, 0, 0, 0, 0, 0});
  if (initial_pose_vec.size() != 6 ||
      !std::all_of(initial_pose_vec.begin(), initial_pose_vec.end(),
                   [](double v) { return std::isfinite(v); })) {
    throw std::invalid_argument("initial_pose must contain six finite base_link pose values");
  }
  try {
    initial_pose_.position.x = initial_pose_vec.at(0);
    initial_pose_.position.y = initial_pose_vec.at(1);
    initial_pose_.position.z = initial_pose_vec.at(2);
    tf2::Quaternion q;
    q.setRPY(initial_pose_vec.at(3), initial_pose_vec.at(4),
             initial_pose_vec.at(5));
    initial_pose_.orientation = tf2::toMsg(q);
  } catch (const std::out_of_range &ex) {
    RCLCPP_ERROR(this->get_logger(),
                 "initial_pose is not a vector with 6 elements, what():%s",
                 ex.what());
  }

  // Set up the pointcloud subscriber
  std::string pointcloud_topic = this->declare_parameter(
      "pointcloud_topic", std::string("/livox/lidar/pointcloud"));
  RCLCPP_INFO(this->get_logger(), "pointcloud_topic: %s",
              pointcloud_topic.c_str());
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, qos,
      std::bind(&IcpNode::pointcloudCallback, this, std::placeholders::_1));
  // Set up the initial pose subscriber
  initial_pose_sub_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", qos,
          [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
            initialPoseCallback(msg);
          });

  // Set up the transform broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tf_publisher_thread_ = std::make_unique<std::thread>([this]() {
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
      {
        std::lock_guard lock(mutex_);
        if (is_ready_) {
            RCLCPP_DEBUG_STREAM(this->get_logger(),
                     "Publishing tf"
                       << map_to_odom_.transform.translation.x << " "
                       << map_to_odom_.transform.translation.y << " "
                       << map_to_odom_.transform.translation.z);
          map_to_odom_.header.stamp = now();
          map_to_odom_.header.frame_id = map_frame_id_;
          map_to_odom_.child_frame_id = odom_frame_id_;
          tf_broadcaster_->sendTransform(map_to_odom_);
        }
      }
      rate.sleep();
    }
  });

  RCLCPP_INFO(this->get_logger(), "icp_registration initialized");
}

IcpNode::~IcpNode() {
  if (tf_publisher_thread_->joinable()) {
    tf_publisher_thread_->join();
  }
}

void IcpNode::pointcloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (msg->header.frame_id != laser_frame_id_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Cloud frame '%s' does not match laser_frame_id '%s'",
                         msg->header.frame_id.c_str(), laser_frame_id_.c_str());
    return;
  }
  pcl::fromROSMsg(*msg, *cloud_in_);
  std::vector<int> indices;
  cloud_in_->is_dense = false;
  pcl::removeNaNFromPointCloud(*cloud_in_, *cloud_in_, indices);
  cloud_stamp_ = rclcpp::Time(msg->header.stamp);
  if (!first_scan_ || cloud_in_->size() < 15) {
    return;
  }
  const auto current = std::chrono::steady_clock::now();
  if (current - last_attempt_ < std::chrono::seconds(2)) {
    return;
  }
  last_attempt_ = current;
  tryInitialize();
}

void IcpNode::initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  const auto &p = msg->pose.pose;
  const auto &q = p.orientation;
  const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
  if (msg->header.frame_id != map_frame_id_ ||
      !std::isfinite(p.position.x) || !std::isfinite(p.position.y) ||
      !std::isfinite(p.position.z) || !std::isfinite(norm) || norm < 1e-12) {
    RCLCPP_ERROR(get_logger(), "Initial base pose must be finite and in the map frame");
    return;
  }
  initial_pose_ = p;
  first_scan_ = true;
  last_attempt_ = std::chrono::steady_clock::time_point{};
  {
    std::lock_guard<std::mutex> lock(mutex_);
    is_ready_ = false;
  }
  // Use the next cloud, whose timestamp follows this user pose request.
}

void IcpNode::tryInitialize() {
  auto matrix = [](const geometry_msgs::msg::TransformStamped &tf) {
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    result.block<3,3>(0,0) = Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
    result.block<3,1>(0,3) = Eigen::Vector3d(t.x,t.y,t.z);
    return result;
  };
  Eigen::Matrix4d base_to_laser, laser_to_odom;
  try {
    // Both transforms refer to the same scan, never to an unrelated now().
    base_to_laser = matrix(tf_buffer_->lookupTransform(
        base_frame_id_, laser_frame_id_, cloud_stamp_, rclcpp::Duration::from_seconds(0.2)));
    laser_to_odom = matrix(tf_buffer_->lookupTransform(
        laser_frame_id_, range_odom_frame_id_, cloud_stamp_, rclcpp::Duration::from_seconds(0.2)));
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "ICP waiting for scan-time TF; will retry: %s", ex.what());
    return;
  }
  const auto &p = initial_pose_;
  const auto &q = p.orientation;
  Eigen::Matrix4d map_to_base = Eigen::Matrix4d::Identity();
  map_to_base.block<3,3>(0,0) = Eigen::Quaterniond(q.w,q.x,q.y,q.z).normalized().toRotationMatrix();
  map_to_base.block<3,1>(0,3) = Eigen::Vector3d(p.position.x,p.position.y,p.position.z);
  const Eigen::Matrix4d initial_guess = map_to_base * base_to_laser;
  RCLCPP_INFO(get_logger(), "Aligning cloud using map-to-base pose and base-to-laser TF");
  const Eigen::Matrix4d map_to_laser = multiAlignSync(cloud_in_, initial_guess);
  if (!success_ || !map_to_laser.allFinite()) {
    RCLCPP_ERROR(get_logger(), "ICP failed; no map-to-odom published, will retry");
    return;
  }
  const Eigen::Matrix4d result = map_to_laser * laser_to_odom;
  if (!result.allFinite()) {
    RCLCPP_ERROR(get_logger(), "Nonfinite ICP transform rejected");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map_to_odom_.transform.translation.x = result(0, 3);
    map_to_odom_.transform.translation.y = result(1, 3);
    map_to_odom_.transform.translation.z = result(2, 3);
    const Eigen::Quaterniond rotation(result.block<3,3>(0,0));
    map_to_odom_.transform.rotation.w = rotation.w();
    map_to_odom_.transform.rotation.x = rotation.x();
    map_to_odom_.transform.rotation.y = rotation.y();
    map_to_odom_.transform.rotation.z = rotation.z();
    is_ready_ = true;
    first_scan_ = false;
  }
  RCLCPP_INFO(get_logger(), "ICP accepted; publishing map-to-odom (score %.6f)", score_);
}

Eigen::Matrix4d IcpNode::multiAlignSync(PointCloudXYZI::Ptr source,
                                        const Eigen::Matrix4d &init_guess) {
  static auto rotate2rpy = [](Eigen::Matrix3d &rot) -> Eigen::Vector3d {
    double roll = std::atan2(rot(2, 1), rot(2, 2));
    double pitch = asin(-rot(2, 0));
    double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
  };

  success_ = false;
  Eigen::Vector3d xyz = init_guess.block<3, 1>(0, 3);
  Eigen::Matrix3d rotation = init_guess.block<3, 3>(0, 0);
  Eigen::Vector3d rpy = rotate2rpy(rotation);
  const int yaw_steps = static_cast<int>(std::floor(yaw_offset_ / yaw_resolution_ + 1e-9));
  std::vector<Eigen::Matrix4f> candidates;
  Eigen::Matrix4f temp_pose;

  RCLCPP_INFO(this->get_logger(), "initial guess: %f, %f, %f, %f, %f, %f",
              xyz(0), xyz(1), xyz(2), rpy(0), rpy(1), rpy(2));

  for (int i = -1; i <= 1; i++) {
    for (int j = -1; j <= 1; j++) {
      for (int k = -yaw_steps; k <= yaw_steps; k++) {
        Eigen::Vector3f pos(xyz(0) + i * xy_offset_, xyz(1) + j * xy_offset_,
                            xyz(2));
        Eigen::AngleAxisf yawAngle(k * yaw_resolution_,
                                   Eigen::Vector3f::UnitZ());
        temp_pose.setIdentity();
        temp_pose.block<3, 3>(0, 0) =
            yawAngle.toRotationMatrix() * rotation.cast<float>();
        temp_pose.block<3, 1>(0, 3) = pos;
        candidates.push_back(temp_pose);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr rough_source(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr refine_source(
      new pcl::PointCloud<pcl::PointXYZI>);

  voxel_rough_filter_.setInputCloud(source);
  voxel_rough_filter_.filter(*rough_source);
  voxel_refine_filter_.setInputCloud(source);
  voxel_refine_filter_.filter(*refine_source);

  PointCloudXYZIN::Ptr rough_source_norm = addNorm(rough_source);
  PointCloudXYZIN::Ptr refine_source_norm = addNorm(refine_source);
  PointCloudXYZIN::Ptr align_point(new PointCloudXYZIN);

  Eigen::Matrix4f best_rough_transform;
  double best_rough_score = 10.0;
  bool rough_converge = false;
  double lowest_rough_score = std::numeric_limits<double>::infinity();
  auto tic = std::chrono::system_clock::now();
  for (Eigen::Matrix4f &init_pose : candidates) {
    icp_rough_.setInputSource(rough_source_norm);
    icp_rough_.align(*align_point, init_pose);
    if (!icp_rough_.hasConverged())
      continue;
    double rough_score = icp_rough_.getFitnessScore();
    if (std::isfinite(rough_score)) lowest_rough_score = std::min(lowest_rough_score, rough_score);
    if (!std::isfinite(rough_score) || rough_score > 2 * thresh_)
      continue;
    if (rough_score < best_rough_score) {
      best_rough_score = rough_score;
      rough_converge = true;
      best_rough_transform = icp_rough_.getFinalTransformation();
    }
  }

  if (!rough_converge) {
    RCLCPP_WARN(get_logger(), "No accepted rough alignment (best score %.6f, limit %.6f)",
                lowest_rough_score, 2 * thresh_);
    return Eigen::Matrix4d::Zero();
  }

  icp_refine_.setInputSource(refine_source_norm);
  icp_refine_.align(*align_point, best_rough_transform);
  score_ = icp_refine_.getFitnessScore();

  if (!icp_refine_.hasConverged() || !std::isfinite(score_) || score_ > thresh_) {
    RCLCPP_WARN(get_logger(), "Refine alignment rejected (score %.6f, limit %.6f)", score_, thresh_);
    return Eigen::Matrix4d::Zero();
  }
  success_ = true;
  auto toc = std::chrono::system_clock::now();
  std::chrono::duration<double> duration = toc - tic;
  RCLCPP_INFO(this->get_logger(), "align used: %f ms", duration.count() * 1000);
  RCLCPP_INFO(this->get_logger(), "score: %f", score_);

  return icp_refine_.getFinalTransformation().cast<double>();
}

PointCloudXYZIN::Ptr
IcpNode::addNorm(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr searchTree(
      new pcl::search::KdTree<pcl::PointXYZI>);
  searchTree->setInputCloud(cloud);

  pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normalEstimator;
  normalEstimator.setInputCloud(cloud);
  normalEstimator.setSearchMethod(searchTree);
  normalEstimator.setKSearch(15);
  normalEstimator.compute(*normals);
  PointCloudXYZIN::Ptr out(new PointCloudXYZIN);
  pcl::concatenateFields(*cloud, *normals, *out);
  return out;
}

} // namespace icp

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(icp::IcpNode)
