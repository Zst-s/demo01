// Global static-map correction from a fresh, map-matched passage mask.
// This layer never changes /map. It must run after StaticLayer and before the
// live TerrainLayer, so a currently observed obstacle can always close a gap.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/static_layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_terrain_bridge {
using Grid = nav_msgs::msg::OccupancyGrid;

class StaticCorrectionLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override {
    auto node = node_.lock();
    if (!node) { throw std::runtime_error("StaticCorrectionLayer lifecycle node expired"); }
    declareParameter("enabled", rclcpp::ParameterValue(false));
    declareParameter("allow_verified_evidence", rclcpp::ParameterValue(false));
    declareParameter("evidence_timeout", rclcpp::ParameterValue(1.0));
    declareParameter("evidence_topic", rclcpp::ParameterValue(
      std::string("/passage/verified_static_clearing")));
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".allow_verified_evidence", allowed_);
    node->get_parameter(name_ + ".evidence_timeout", timeout_);
    node->get_parameter(name_ + ".evidence_topic", evidence_topic_);
    if (!std::isfinite(timeout_) || timeout_ <= 0.0 || timeout_ > 5.0) {
      throw std::invalid_argument("StaticCorrectionLayer evidence_timeout must be in (0,5]");
    }
    if (enabled_ && !allowed_) {
      throw std::invalid_argument("StaticCorrectionLayer requires explicit verified-evidence opt-in");
    }
    if (evidence_topic_ != "/passage/verified_static_clearing" &&
        !(evidence_topic_ == "/passage_fusion_test/preview_static_clearing" &&
          std::string(node->get_namespace()) == "/passage_fusion_test")) {
      throw std::invalid_argument("StaticCorrectionLayer refuses unapproved evidence topic");
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    map_sub_ = node->create_subscription<Grid>(
      "/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](Grid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!validGrid(*msg, false)) {
          map_.reset(); mask_.reset(); return;
        }
        if (!map_ || !sameGeometry(*map_, *msg) ||
            map_->info.map_load_time != msg->info.map_load_time) {
          // A new map association revokes the old geometry. Same-canvas SLAM
          // occupancy updates retain the structure mask; updateCosts intersects
          // it with the current static map, and the live terrain layer runs next.
          mask_.reset(); high_water_stamp_ = 0;
        }
        map_ = msg;
      }, options);
    mask_sub_ = node->create_subscription<Grid>(
      evidence_topic_,
      rclcpp::QoS(1).reliable().durability_volatile(),
      [this](Grid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!validGrid(*msg, true) || !map_ ||
            !sameGeometry(*map_, *msg) ||
            map_->info.map_load_time != msg->info.map_load_time) {
          mask_.reset(); return;
        }
        const int64_t stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (stamp <= high_water_stamp_ || age < -0.1 || age > timeout_) {
          mask_.reset(); return;
        }
        high_water_stamp_ = stamp;
        mask_ = msg;
        received_ = std::chrono::steady_clock::now();
      }, options);
    current_ = true;  // No proof means the unchanged static wall remains valid.
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    mask_.reset(); working_.reset(); map_working_.reset();
    // previous_ is retained so the next update restores the old open area.
  }
  void matchSize() override { reset(); }
  void deactivate() override { reset(); }
  bool isClearable() override { return false; }

  void updateBounds(double, double, double, double *min_x, double *min_y,
                    double *max_x, double *max_y) override {
    std::lock_guard<std::mutex> lock(mutex_);
    working_.reset(); map_working_.reset();
    if (have_previous_) { expand(previous_, min_x, min_y, max_x, max_y); }
    have_previous_ = false;
    auto node = node_.lock();
    bool enabled_now = false;
    if (node) { node->get_parameter(name_ + ".enabled", enabled_now); }
    enabled_ = enabled_now && allowed_;
    const auto now = clock_->now();
    if (last_now_ != 0 && now.nanoseconds() < last_now_) {
      map_.reset(); mask_.reset(); high_water_stamp_ = 0;
    }
    last_now_ = now.nanoseconds();
    if (!enabled_ || !validOrder() || !map_ || !mask_) { return; }
    const double age = (now - rclcpp::Time(mask_->header.stamp)).seconds();
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_).count();
    if (age < -0.1 || age > timeout_ || receive_age > timeout_ ||
        !matchesMaster(*map_)) {
      mask_.reset(); return;
    }
    const auto &info = map_->info;
    previous_ = {info.origin.position.x, info.origin.position.y,
      info.origin.position.x + info.width * info.resolution,
      info.origin.position.y + info.height * info.resolution};
    expand(previous_, min_x, min_y, max_x, max_y);
    have_previous_ = true;
    working_ = mask_;
    map_working_ = map_;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D &master, int min_i, int min_j,
                   int max_i, int max_j) override {
    if (!enabled_ || !working_ || !map_working_ || !validOrder()) { return; }
    for (int y = min_j; y < max_j; ++y) {
      for (int x = min_i; x < max_i; ++x) {
        const size_t index = static_cast<size_t>(y) * working_->info.width + x;
        if (index >= working_->data.size()) { continue; }
        if (working_->data[index] == 100 && map_working_->data[index] == 100 &&
            master.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          master.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        }
      }
    }
  }

private:
  bool validOrder() {
    const auto &plugins = *layered_costmap_->getPlugins();
    const bool ordered = !layered_costmap_->isRolling() &&
      layered_costmap_->getGlobalFrameID() == "map" &&
      layered_costmap_->getFilters()->empty() && plugins.size() == 4 &&
      dynamic_cast<nav2_costmap_2d::StaticLayer *>(plugins[0].get()) &&
      plugins[0]->isEnabled() && plugins[1].get() == this &&
      plugins[2]->getName() == "terrain_layer" && plugins[2]->isEnabled() &&
      dynamic_cast<nav2_costmap_2d::InflationLayer *>(plugins[3].get()) &&
      plugins[3]->isEnabled();
    auto node = node_.lock();
    std::string terrain_type;
    if (node) { node->get_parameter("terrain_layer.plugin", terrain_type); }
    if (!ordered || terrain_type != "sentry_terrain_bridge::TerrainLayer") {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000,
                            "Unsafe layer order; static correction disabled");
      return false;
    }
    return true;
  }

  static bool validGrid(const Grid &grid, bool mask) {
    const auto &info = grid.info;
    const auto &position = info.origin.position;
    const auto &orientation = info.origin.orientation;
    const size_t count = static_cast<size_t>(info.width) * info.height;
    return grid.header.frame_id == "map" &&
      grid.header.stamp.sec >= 0 && grid.header.stamp.nanosec < 1000000000U &&
      info.map_load_time.sec >= 0 && info.map_load_time.nanosec < 1000000000U &&
      info.width > 0 && info.height > 0 && count <= 4000000 &&
      grid.data.size() == count && std::isfinite(info.resolution) &&
      info.resolution > 0 && std::isfinite(position.x) &&
      std::isfinite(position.y) && position.z == 0.0 &&
      orientation.x == 0.0 && orientation.y == 0.0 &&
      orientation.z == 0.0 && orientation.w == 1.0 &&
      std::all_of(grid.data.begin(), grid.data.end(), [mask](int8_t value) {
        return mask ? (value == 0 || value == 100) :
                      (value >= -1 && value <= 100);
      });
  }

  static bool sameGeometry(const Grid &a, const Grid &b) {
    return a.header.frame_id == b.header.frame_id &&
      a.info.width == b.info.width && a.info.height == b.info.height &&
      a.info.resolution == b.info.resolution && a.info.origin == b.info.origin;
  }

  bool matchesMaster(const Grid &grid) const {
    const auto *master = layered_costmap_->getCostmap();
    return grid.info.width == master->getSizeInCellsX() &&
      grid.info.height == master->getSizeInCellsY() &&
      std::abs(grid.info.resolution - master->getResolution()) < 1e-7 &&
      std::abs(grid.info.origin.position.x - master->getOriginX()) < 1e-7 &&
      std::abs(grid.info.origin.position.y - master->getOriginY()) < 1e-7;
  }

  static void expand(const std::array<double, 4> &bounds, double *min_x,
                     double *min_y, double *max_x, double *max_y) {
    *min_x = std::min(*min_x, bounds[0]);
    *min_y = std::min(*min_y, bounds[1]);
    *max_x = std::max(*max_x, bounds[2]);
    *max_y = std::max(*max_y, bounds[3]);
  }

  rclcpp::Subscription<Grid>::SharedPtr map_sub_, mask_sub_;
  std::mutex mutex_;
  Grid::ConstSharedPtr map_, mask_, map_working_, working_;
  std::chrono::steady_clock::time_point received_{};
  std::array<double, 4> previous_{};
  bool have_previous_{false}, allowed_{false};
  double timeout_{1.0};
  std::string evidence_topic_;
  int64_t high_water_stamp_{0}, last_now_{0};
};
}  // namespace sentry_terrain_bridge

PLUGINLIB_EXPORT_CLASS(sentry_terrain_bridge::StaticCorrectionLayer,
                       nav2_costmap_2d::Layer)
