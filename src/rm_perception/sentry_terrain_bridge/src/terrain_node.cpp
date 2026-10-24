#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <rog_map/query_adapter.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <sentry_terrain_bridge/grid_contract.hpp>
#include <sentry_terrain_bridge/grid_scan.hpp>
#include <sentry_terrain_bridge/clearance_observer.hpp>
#include <sentry_terrain_bridge/hollow_structure.hpp>

namespace sentry_terrain_bridge {
// 单线程执行器串行处理点云、里程计及地图更新，不与原 LIO 争抢状态，也不发布
// TF。
class TerrainBridge {
public:
  explicit TerrainBridge(const rclcpp::Node::SharedPtr &node) : node_(node) {
    const auto cloud_topic = node_->declare_parameter<std::string>(
        "cloud_topic", "/cloud_registered_body");
    const auto odom_topic =
        node_->declare_parameter<std::string>("odom_topic", "/Odometry");
    input_frame_ =
        node_->declare_parameter<std::string>("input_frame", "livox_frame");
    cloud_coordinates_ =
        node_->declare_parameter<std::string>("cloud_coordinates", "imu");
    if (cloud_coordinates_ != "imu" && cloud_coordinates_ != "odom") {
      throw std::invalid_argument("cloud_coordinates 只能为 imu 或 odom");
    }
    max_age_ = node_->declare_parameter<double>("max_input_age", 0.5);
    max_delta_ = node_->declare_parameter<double>("max_cloud_odom_dt", 0.10);
    const auto offset = node_->declare_parameter<std::vector<double>>(
        "sensor_offset", {0., 0., 0.});
    if (!(max_age_ > 0.0) || !(max_delta_ > 0.0) || offset.size() != 3) {
      throw std::invalid_argument(
          "时间门限必须大于零，sensor_offset 必须为三个数值");
    }
    sensor_offset_ = Eigen::Vector3d(offset[0], offset[1], offset[2]);
    if (!sensor_offset_.allFinite()) {
      throw std::invalid_argument("sensor_offset 包含非有限值");
    }
    self_filter_enabled_ = node_->declare_parameter<bool>("self_filter.enabled", false);
    if (self_filter_enabled_) {
      const auto limits = node_->declare_parameter<std::vector<double>>(
          "self_filter.box", std::vector<double>{});
      if (limits.size() != 6 ||
          !std::all_of(limits.begin(), limits.end(),
                       [](double value) { return std::isfinite(value); }) ||
          limits[0] >= limits[1] || limits[2] >= limits[3] ||
          limits[4] >= limits[5]) {
        throw std::invalid_argument("self_filter.box 需要有限且递增的 [xmin,xmax,ymin,ymax,zmin,zmax]");
      }
      self_filter_box_ = limits;
    }
    rog_map::Config config;
    config.loadFromRosNode(node_, "rog_map");
    // 禁止同时启用原库订阅，所有数据必须先经过本适配器的坐标系和时间检查。
    if (config.ros_callback_en || !config.layer_en || config.prior_map_enable ||
        config.load_pcd_en) {
      throw std::invalid_argument("要求 "
                                  "ros_callback=false、projection=true、prior_"
                                  "map=false、load_pcd_en=false");
    }
    frame_ = config.frame_id;
    if (frame_.empty()) {
      throw std::invalid_argument("rog_map.frame_id 不能为空");
    }
    if (cloud_coordinates_ == "odom" && input_frame_ != frame_) {
      throw std::invalid_argument(
          "odom 点云的 input_frame 必须等于 rog_map.frame_id");
    }
    map_ = std::make_unique<rog_map::ROGMapROS>(node_, config);
    if (node_->declare_parameter<bool>("clearance.enabled", false)) {
      clearance_ = std::make_unique<ClearanceObserver>(node_, *map_, frame_);
    }
    query_ = std::dynamic_pointer_cast<rog_map::QueryAdapter>(
        map_->queryInterface());
    if (!query_) {
      throw std::runtime_error("ROGMap 没有提供 QueryAdapter 快照接口");
    }
    // 发布标准 OccupancyGrid；并非导航静态地图 /map，不提供 map->odom。
    grid_pub_ = node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/terrain/grid", rclcpp::QoS(1).reliable().durability_volatile());
    structure_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/terrain/passage/structure", rclcpp::QoS(1).reliable().durability_volatile());
    mapping_scan_enabled_ =
        node_->declare_parameter<bool>("mapping_scan_enabled", false);
    if (mapping_scan_enabled_) {
      mapping_scan_frame_ = node_->declare_parameter<std::string>(
          "mapping_scan_frame", "livox_frame");
      mapping_scan_range_min_ = static_cast<float>(
          node_->declare_parameter<double>("mapping_scan_range_min", 0.48));
      mapping_scan_range_max_ = static_cast<float>(
          node_->declare_parameter<double>("mapping_scan_range_max", 4.5));
      mapping_scan_angle_increment_ = static_cast<float>(
          node_->declare_parameter<double>("mapping_scan_angle_increment", 0.00872664626));
      mapping_scan_time_ = static_cast<float>(
          node_->declare_parameter<double>("mapping_scan_time", 0.1));
      if (mapping_scan_frame_.empty() ||
          !(mapping_scan_range_min_ > 0.0F) ||
          !(mapping_scan_range_max_ > mapping_scan_range_min_) ||
          !(mapping_scan_angle_increment_ > 0.0F) ||
          !(mapping_scan_time_ > 0.0F)) {
        throw std::invalid_argument("ROGMap 建图扫描配置无效");
      }
      mapping_scan_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
          "/rog_map/scan", rclcpp::SensorDataQoS().keep_last(1));
    }
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS().keep_last(100),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          if (msg->header.frame_id != frame_) {
            RCLCPP_ERROR_THROTTLE(
                node_->get_logger(), *node_->get_clock(), 2000,
                "里程计 frame=%s，期望 %s；拒绝仅改 frame_id 冒充坐标变换",
                msg->header.frame_id.c_str(), frame_.c_str());
            return;
          }
          if (!odoms_.empty() &&
              rclcpp::Time(msg->header.stamp) <
                  rclcpp::Time(odoms_.back()->header.stamp)) {
            odoms_.clear(); // 时间回退后不拿旧里程计匹配新点云。
          }
          odoms_.push_back(msg);
          while (odoms_.size() > 200) {
            odoms_.pop_front();
          }
        });
    cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          if (msg->header.frame_id != input_frame_) {
            RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(),
                                  2000, "点云 frame=%s，期望 %s",
                                  msg->header.frame_id.c_str(),
                                  input_frame_.c_str());
            return;
          }
          pending_cloud_ =
              msg; // 只保留最新点云，防止积压后机器人还在使用数秒前的障碍物。
        });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(20),
                                      [this]() { process(); });
    RCLCPP_INFO(
        node_->get_logger(),
        "ROGMap 感知：%s + %s -> /terrain/grid，frame=%s；不发布速度或 TF",
        cloud_topic.c_str(), odom_topic.c_str(), frame_.c_str());
  }

private:
  void process() {
    if (!pending_cloud_) {
      return;
    }
    const auto stamp = rclcpp::Time(pending_cloud_->header.stamp);
    const double age = (node_->now() - stamp).seconds();
    if (age < -max_delta_ || age > max_age_) {
      RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 2000,
          "丢弃点云：时间差 %.3f s，请检查 use_sim_time、时间同步或计算延迟",
          age);
      pending_cloud_.reset();
      return;
    }
    if (odoms_.empty()) {
      return;
    }
    auto best = odoms_.front();
    double delta = std::numeric_limits<double>::infinity();
    for (const auto &odom : odoms_) {
      const double dt =
          std::abs((rclcpp::Time(odom->header.stamp) - stamp).seconds());
      if (dt < delta) {
        delta = dt;
        best = odom;
      }
    }
    if (delta > max_delta_) {
      return;
    } // 等待同一时刻的里程计，不直接使用“最近收到”的旧位姿。
    auto cloud = pending_cloud_;
    pending_cloud_.reset();
    const auto &p = best->pose.pose.position;
    const auto &q = best->pose.pose.orientation;
    Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    Eigen::Vector3d position(p.x, p.y, p.z);
    if (!position.allFinite() || !rotation.coeffs().allFinite() ||
        rotation.norm() < 1e-6) {
      return;
    }
    rotation.normalize();
    // 本仓库两种 LIO 的 Odometry 实际写入 IMU 状态；雷达射线起点需加
    // R*extrinsic_T。
    const Eigen::Vector3d sensor_position =
        position + rotation * sensor_offset_;
    if (last_stamp_.nanoseconds() != 0 && stamp <= last_stamp_) {
      RCLCPP_ERROR(
          node_->get_logger(),
          "点云时间回退或重复：请重启感知节点，不能混用重置前后的地图");
      return;
    }
    pcl::PointCloud<pcl::PointXYZ> xyz;
    pcl::fromROSMsg(*cloud, xyz);
    rog_map::PointCloud points;
    points.reserve(xyz.size());
    for (const auto &point : xyz) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      rog_map::PclPoint out;
      Eigen::Vector3d world(point.x, point.y, point.z);
      // Remove known chassis returns before raycasting. The box is in the
      // odometry/IMU body frame, not in map coordinates, and is disabled by
      // default until the actual vehicle's own geometry is calibrated.
      if (self_filter_enabled_) {
        const Eigen::Vector3d local = cloud_coordinates_ == "imu"
            ? world : rotation.conjugate() * (world - position);
        if (local.x() >= self_filter_box_[0] && local.x() <= self_filter_box_[1] &&
            local.y() >= self_filter_box_[2] && local.y() <= self_filter_box_[3] &&
            local.z() >= self_filter_box_[4] && local.z() <= self_filter_box_[5]) {
          continue;
        }
      }
      // 经源码核实：本仓库 /cloud_registered_body 已转到 IMU 系，虽然 header 写
      // livox_frame。 因此只乘 odom<-IMU 位姿，不能再应用一次雷达-IMU
      // 外参或通过这个错误标签查询 TF。 若改用 /cloud_registered，需显式选择
      // odom 模式，避免重复变换。
      if (cloud_coordinates_ == "imu") {
        world = rotation * world + position;
      }
      out.x = world.x();
      out.y = world.y();
      out.z = world.z();
      out.intensity =
          0.0f; // 地形几何不依赖反射强度，兼容没有 intensity 字段的点云。
      points.push_back(out);
    }
    if (points.empty()) {
      return;
    }
    map_->updateMap(points, super_utils::Pose(sensor_position, rotation));
    if (clearance_) { clearance_->inputUpdated(stamp.seconds()); }
    const auto snapshot = query_->snapshot();
    if (!snapshot || snapshot->width <= 0 || snapshot->height <= 0 ||
        snapshot->values.size() !=
            static_cast<size_t>(snapshot->width) * snapshot->height) {
      return;
    }
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp =
        cloud->header.stamp; // 保留数据时间，不能把延迟数据伪装成当前数据。
    grid.header.frame_id = frame_;
    grid.info.resolution = static_cast<float>(snapshot->resolution);
    grid.info.width = snapshot->width;
    grid.info.height = snapshot->height;
    grid.info.origin.position.x = snapshot->origin_x;
    grid.info.origin.position.y = snapshot->origin_y;
    grid.info.origin.orientation.w = 1.0;
    grid.data.reserve(snapshot->values.size());
    for (const auto value : snapshot->values) {
      grid.data.push_back(toOccupancy(value));
    }
    grid_pub_->publish(grid);
    if (const auto layer = map_->getProjectionLayer()) {
      structure_pub_->publish(hollowStructureCloud(*layer, frame_, cloud->header.stamp));
    }
    if (mapping_scan_enabled_) {
      // /Odometry 为 IMU 姿态；本车雷达外参旋转是单位矩阵，因此其 yaw 相同。
      // 扫描原点采用已经用于 ROGMap 射线更新的雷达位置，不用 odom 原点。
      const double yaw = std::atan2(
          2.0 * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
          1.0 - 2.0 * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));
      auto scan = projectGridToScan(
          grid, sensor_position.x(), sensor_position.y(), yaw,
          mapping_scan_frame_, mapping_scan_range_min_, mapping_scan_range_max_,
          mapping_scan_angle_increment_, mapping_scan_time_);
      if (hasUsableMappingReturn(scan)) {
        mapping_scan_pub_->publish(scan);
      } else {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 5000,
            "ROGMap 扫描没有有效障碍回波，暂不送入 SLAM，避免初始化为空地图；地形栅格仍正常发布");
      }
    }
    last_stamp_ = stamp;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<rog_map::ROGMapROS> map_;
  std::unique_ptr<ClearanceObserver> clearance_;
  std::shared_ptr<rog_map::QueryAdapter> query_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr structure_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr mapping_scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::deque<nav_msgs::msg::Odometry::ConstSharedPtr> odoms_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending_cloud_;
  Eigen::Vector3d sensor_offset_;
  std::vector<double> self_filter_box_;
  std::string frame_, input_frame_, cloud_coordinates_;
  double max_age_, max_delta_;
  bool self_filter_enabled_{false};
  bool mapping_scan_enabled_{false};
  std::string mapping_scan_frame_;
  float mapping_scan_range_min_{0.48F}, mapping_scan_range_max_{4.5F};
  float mapping_scan_angle_increment_{0.00872664626F}, mapping_scan_time_{0.1F};
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
};
} // namespace sentry_terrain_bridge

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sentry_terrain");
  try {
    sentry_terrain_bridge::TerrainBridge bridge(node);
    rclcpp::spin(node); // 使用单线程执行器，禁止在此替换为多线程 spin。
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "地形感知退出：%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
