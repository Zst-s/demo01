#include <memory>
#include <chrono>
#include <cmath>
#include <stdexcept>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

class SegmentationNode : public rclcpp::Node {
public:
  SegmentationNode(const rclcpp::NodeOptions &node_options);
  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  GroundSegmentationParams params_;
  std::shared_ptr<GroundSegmentation> segmenter_;
  std::string gravity_aligned_frame_;
  sensor_msgs::msg::PointCloud2::SharedPtr pending_cloud_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  double max_input_age_;
};

SegmentationNode::SegmentationNode(const rclcpp::NodeOptions &node_options)
    : Node("ground_segmentation", node_options) {
  gravity_aligned_frame_ =
      this->declare_parameter("gravity_aligned_frame", "gravity_aligned");
  max_input_age_ = this->declare_parameter("max_input_age", 0.5);
  if (!std::isfinite(max_input_age_) || max_input_age_ <= 0.) {
    throw std::invalid_argument("max_input_age must be positive and finite");
  }

  params_.visualize = this->declare_parameter("visualize", params_.visualize);
  params_.n_bins = this->declare_parameter("n_bins", params_.n_bins);
  params_.n_segments =
      this->declare_parameter("n_segments", params_.n_segments);
  params_.max_dist_to_line =
      this->declare_parameter("max_dist_to_line", params_.max_dist_to_line);
  params_.max_slope = this->declare_parameter("max_slope", params_.max_slope);
  params_.min_slope = this->declare_parameter("min_slope", params_.min_slope);
  params_.long_threshold =
      this->declare_parameter("long_threshold", params_.long_threshold);
  params_.max_long_height =
      this->declare_parameter("max_long_height", params_.max_long_height);
  params_.max_start_height =
      this->declare_parameter("max_start_height", params_.max_start_height);
  params_.sensor_height =
      this->declare_parameter("sensor_height", params_.sensor_height);
  params_.line_search_angle =
      this->declare_parameter("line_search_angle", params_.line_search_angle);
  params_.n_threads = this->declare_parameter("n_threads", params_.n_threads);
  // Existing YAML files mix integer ranges and floating-point distances.
  const auto declare_numeric = [this](const std::string &name, double fallback) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    const auto value = this->declare_parameter(
        name, rclcpp::ParameterValue(fallback), descriptor);
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
      return static_cast<double>(value.get<int64_t>());
    }
    if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
      throw std::invalid_argument(name + " must be numeric");
    }
    return value.get<double>();
  };
  const double r_min = declare_numeric("r_min", std::sqrt(params_.r_min_square));
  const double r_max = declare_numeric("r_max", std::sqrt(params_.r_max_square));
  const double max_fit_error = declare_numeric(
      "max_fit_error", std::sqrt(params_.max_error_square));
  if (!std::isfinite(r_min) || !std::isfinite(r_max) || !std::isfinite(max_fit_error) ||
      r_min < 0.0 || r_max <= r_min || max_fit_error <= 0.0) {
    throw std::invalid_argument("Require finite 0 <= r_min < r_max and max_fit_error > 0");
  }
  params_.r_min_square = r_min * r_min;
  params_.r_max_square = r_max * r_max;
  params_.max_error_square = max_fit_error * max_fit_error;
  segmenter_ = std::make_shared<GroundSegmentation>(params_);
  std::string ground_topic, obstacle_topic, input_topic;
  ground_topic = this->declare_parameter("ground_output_topic", "ground_cloud");
  obstacle_topic =
      this->declare_parameter("obstacle_output_topic", "obstacle_cloud");
  input_topic = this->declare_parameter("input_topic", "input_cloud");
  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { pending_cloud_ = msg; });
  ground_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      ground_topic, rclcpp::SensorDataQoS());
  obstacle_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      obstacle_topic, rclcpp::SensorDataQoS());
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  retry_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this]() {
    if (!pending_cloud_) { return; }
    const double age = (this->now()-rclcpp::Time(pending_cloud_->header.stamp)).seconds();
    if (age < -0.1 || age > max_input_age_) {
      pending_cloud_.reset();
      return;
    }
    scanCallback(pending_cloud_);
  });
  RCLCPP_INFO(this->get_logger(), "Segmentation node initialized");
}

void SegmentationNode::scanCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*msg, cloud);
  pcl::PointCloud<pcl::PointXYZ> cloud_transformed;

  std::vector<int> labels;

  bool is_original_pc = true;
  if (!gravity_aligned_frame_.empty()) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_->lookupTransform(
          gravity_aligned_frame_, msg->header.frame_id, msg->header.stamp);
      // Remove translation part.
      tf_stamped.transform.translation.x = 0;
      tf_stamped.transform.translation.y = 0;
      tf_stamped.transform.translation.z = 0;
      Eigen::Affine3d tf = Eigen::Affine3d::Identity();
      tf.translate(Eigen::Vector3d(0, 0, 0));
      tf.rotate(Eigen::Quaterniond(
          tf_stamped.transform.rotation.w, tf_stamped.transform.rotation.x,
          tf_stamped.transform.rotation.y, tf_stamped.transform.rotation.z));
      // tf::transformMsgToEigen(tf_stamped.transform, tf);
      pcl::transformPointCloud(cloud, cloud_transformed, tf);
      is_original_pc = false;
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                  "Failed to transform point cloud into "
                  "gravity frame: %s",
                  ex.what());
      return;  // Wait for timestamped TF; never classify a tilted raw cloud as level.
    }
  }

  // Trick to avoid PC copy if we do not transform.
  const pcl::PointCloud<pcl::PointXYZ> &cloud_proc =
      is_original_pc ? cloud : cloud_transformed;

  segmenter_->segment(cloud_proc, &labels);
  pcl::PointCloud<pcl::PointXYZ> ground_cloud, obstacle_cloud;
  for (size_t i = 0; i < cloud.size(); ++i) {
    if (labels[i] == 1)
      ground_cloud.push_back(cloud[i]);
    else
      obstacle_cloud.push_back(cloud[i]);
  }
  auto ground_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  auto obstacle_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(ground_cloud, *ground_msg);
  pcl::toROSMsg(obstacle_cloud, *obstacle_msg);
  ground_msg->header = msg->header;
  obstacle_msg->header = msg->header;
  ground_pub_->publish(*ground_msg);
  obstacle_pub_->publish(*obstacle_msg);
  pending_cloud_.reset();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<SegmentationNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
