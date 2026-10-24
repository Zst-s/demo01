// 第一阶段实验插件：人工掩码不代表真实三维通行证据。
// 不修改 /map，不发布 TF 或速度；仅允许专用测试链路使用。
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/static_layer.hpp>
#include <nav2_costmap_2d/inflation_layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace sentry_passage_fusion_test {
using Grid = nav_msgs::msg::OccupancyGrid;
class StaticCorrectionLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override {
    auto node = node_.lock();
    const std::string ns = node ? std::string(node->get_namespace()) : "";
    if (ns != "/passage_fusion_test" &&
        ns != "/passage_fusion_test/global_costmap") {
      throw std::runtime_error("TEST ONLY: namespace must be the isolated test costmap");
    }
    declareParameter("enabled", rclcpp::ParameterValue(false));
    declareParameter("allow_test_evidence", rclcpp::ParameterValue(false));
    declareParameter("evidence_timeout", rclcpp::ParameterValue(1.0));
    bool test_allowed = false;
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".allow_test_evidence", test_allowed);
    node->get_parameter(name_ + ".evidence_timeout", timeout_);
    if (!test_allowed || !std::isfinite(timeout_) || timeout_ <= 0.0 || timeout_ > 5.0) {
      throw std::runtime_error("Explicit test opt-in and evidence_timeout in (0,5] required");
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    map_sub_ = node->create_subscription<Grid>("/passage_fusion_test/map", rclcpp::QoS(1).reliable().transient_local(),
      [this](Grid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!validGrid(*msg, false)) {
          map_.reset();
          evidence_.reset();
          return;
        }
        // 本测试协议中 map_load_time 是地图版本；新地图必须撤销旧证据。
        if (!map_ || !sameGeometry(*map_, *msg) ||
            map_->info.map_load_time != msg->info.map_load_time || map_->data != msg->data) {
          evidence_.reset();
        }
        map_ = msg;
      }, options);
    evidence_sub_ = node->create_subscription<Grid>(
      "/passage_fusion_test/test_evidence", rclcpp::QoS(1).reliable().durability_volatile(),
      [this](Grid::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // 先检查 ROS 时间字段，再构造 rclcpp::Time，防止畸形消息抛异常终止宿主。
        if (!validGrid(*msg, true)) {
          evidence_.reset();
          return;
        }
        const auto stamp = rclcpp::Time(msg->header.stamp).nanoseconds();
        const double age = (clock_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (!enabled_ || !map_ || !sameGeometry(*map_, *msg) ||
            msg->info.map_load_time != map_->info.map_load_time ||
            stamp <= high_water_stamp_ || age < 0.0 || age > timeout_) {
          // 无效输入不得延长旧许可；下一轮恢复原始静态地图。
          evidence_.reset();
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Rejected test evidence; authorization revoked");
          return;
        }
        high_water_stamp_ = stamp;
        evidence_ = msg;  // 全量快照：新快照中没标记的格子不继承旧许可。
        received_ = std::chrono::steady_clock::now();
      }, options);
    current_ = true;  // 无证据时保留静态墙，是正常的保守状态。
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    evidence_.reset();
    working_.reset();
    // 保留旧区域边界，保证下一轮不会遗留已经打开的洞。
  }
  void matchSize() override { reset(); }
  void deactivate() override { reset(); }
  bool isClearable() override { return false; }

  void updateBounds(double, double, double, double *min_x, double *min_y,
                    double *max_x, double *max_y) override {
    std::lock_guard<std::mutex> lock(mutex_);
    working_.reset();
    map_working_.reset();
    if (have_previous_) { expand(previous_, min_x, min_y, max_x, max_y); }
    have_previous_ = false;
    auto node = node_.lock();
    bool enabled_now = false;
    if (node) { node->get_parameter(name_ + ".enabled", enabled_now); }
    enabled_ = enabled_now;
    const auto now = clock_->now();
    if (last_now_ != 0 && now.nanoseconds() < last_now_) {
      // 时间回退后要求重新接收地图与证据，不能复用旧许可。
      evidence_.reset();
      map_.reset();
      high_water_stamp_ = 0;
    }
    last_now_ = now.nanoseconds();
    if (!enabled_ || !validOrder()) { evidence_.reset(); return; }
    if (!map_ || !evidence_) { return; }
    const double age = (now - rclcpp::Time(evidence_->header.stamp)).seconds();
    const double receive_age = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - received_).count();
    if (age < 0.0 || age > timeout_ || receive_age > timeout_ || !matchesMaster(*map_)) {
      evidence_.reset();
      return;
    }
    // 第一阶段采用小地图全范围重算；优先验证恢复正确性，不做增量优化。
    const auto &info = map_->info;
    previous_ = {info.origin.position.x, info.origin.position.y,
      info.origin.position.x + info.width * info.resolution,
      info.origin.position.y + info.height * info.resolution};
    expand(previous_, min_x, min_y, max_x, max_y);
    have_previous_ = true;
    working_ = evidence_;
    map_working_ = map_;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D &master, int min_i, int min_j,
                   int max_i, int max_j) override {
    if (!enabled_ || !working_ || !map_working_ || !validOrder()) { return; }
    // 使用本轮不可变快照，避免回调在同一更新周期内改变清障范围。
    for (int y = min_j; y < max_j; ++y) {
      for (int x = min_i; x < max_i; ++x) {
        const auto i = static_cast<size_t>(y) * working_->info.width + x;
        if (i >= working_->data.size()) { continue; }
        // 只修正明确的静态占据；不把未知格清成自由空间。
        if (working_->data[i] == 100 && map_working_->data[i] == 100 &&
            master.getCost(x, y) == nav2_costmap_2d::LETHAL_OBSTACLE) {
          master.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
        }
      }
    }
  }

private:
  bool validOrder() {
    const auto &p = *layered_costmap_->getPlugins();
    // Master 不存障碍来源，故严格约束测试图层顺序，禁止清除后面的实时障碍。
    const bool valid = !layered_costmap_->isRolling() &&
      layered_costmap_->getGlobalFrameID() == "map" &&
      layered_costmap_->getFilters()->empty() && p.size() == 4 &&
      dynamic_cast<nav2_costmap_2d::StaticLayer *>(p[0].get()) &&
      p[0]->isEnabled() && p[1].get() == this && p[2]->getName() == "terrain_layer" &&
      p[2]->isEnabled() && dynamic_cast<nav2_costmap_2d::InflationLayer *>(p[3].get()) &&
      p[3]->isEnabled();
    auto node = node_.lock();
    std::string terrain_type;
    if (node) { node->get_parameter("terrain_layer.plugin", terrain_type); }
    if (!valid || terrain_type != "sentry_terrain_bridge::TerrainLayer") {
      RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000, "Unsafe test layer order; correction disabled");
      return false;
    }
    return true;
  }

  static bool validGrid(const Grid &g, bool evidence) {
    const auto &i = g.info;
    const auto &p = i.origin.position;
    const auto &q = i.origin.orientation;
    const size_t n = static_cast<size_t>(i.width) * i.height;
    return g.header.frame_id == "map" &&
      g.header.stamp.sec >= 0 && g.header.stamp.nanosec < 1000000000U &&
      i.map_load_time.sec >= 0 && i.map_load_time.nanosec < 1000000000U &&
      i.width > 0 && i.height > 0 &&
      n <= 1000000 && g.data.size() == n && std::isfinite(i.resolution) &&
      i.resolution > 0 && std::isfinite(p.x) && std::isfinite(p.y) && p.z == 0.0 &&
      q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 1.0 &&
      (i.map_load_time.sec != 0 || i.map_load_time.nanosec != 0) &&
      std::all_of(g.data.begin(), g.data.end(), [evidence](int8_t v) {
        return evidence ? (v == -1 || v == 0 || v == 100) : (v >= -1 && v <= 100);
      });
  }
  static bool sameGeometry(const Grid &a, const Grid &b) {
    return a.header.frame_id == b.header.frame_id &&
      a.info.width == b.info.width && a.info.height == b.info.height &&
      a.info.resolution == b.info.resolution && a.info.origin == b.info.origin;
  }
  bool matchesMaster(const Grid &g) const {
    const auto *m = layered_costmap_->getCostmap();
    return g.info.width == m->getSizeInCellsX() && g.info.height == m->getSizeInCellsY() &&
      std::abs(g.info.resolution - m->getResolution()) < 1e-7 &&
      std::abs(g.info.origin.position.x - m->getOriginX()) < 1e-7 &&
      std::abs(g.info.origin.position.y - m->getOriginY()) < 1e-7;
  }
  static void expand(const std::array<double, 4> &b, double *x0, double *y0,
                     double *x1, double *y1) {
    *x0 = std::min(*x0, b[0]); *y0 = std::min(*y0, b[1]);
    *x1 = std::max(*x1, b[2]); *y1 = std::max(*y1, b[3]);
  }
  rclcpp::Subscription<Grid>::SharedPtr map_sub_, evidence_sub_;
  std::mutex mutex_;
  Grid::ConstSharedPtr map_, evidence_, map_working_, working_;
  std::chrono::steady_clock::time_point received_{};
  std::array<double, 4> previous_{};
  bool have_previous_{false};
  double timeout_{1.0};
  int64_t high_water_stamp_{0}, last_now_{0};
};
}  // namespace sentry_passage_fusion_test
PLUGINLIB_EXPORT_CLASS(sentry_passage_fusion_test::StaticCorrectionLayer, nav2_costmap_2d::Layer)
