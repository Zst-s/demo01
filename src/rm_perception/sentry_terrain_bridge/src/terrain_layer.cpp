#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <nav2_costmap_2d/layer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sentry_terrain_bridge/grid_contract.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>

namespace sentry_terrain_bridge {
// 使用完整地形快照，不把每帧栅格当成静态地图，避免覆盖 /map 或改变主地图大小。
class TerrainLayer : public nav2_costmap_2d::Layer {
public:
  void onInitialize() override {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("TerrainLayer lifecycle node expired");
    }
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("topic",
                     rclcpp::ParameterValue(std::string("/terrain/grid")));
    declareParameter("data_timeout", rclcpp::ParameterValue(0.7));
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".data_timeout", timeout_);
    std::string topic;
    node->get_parameter(name_ + ".topic", topic);
    if (!(timeout_ > 0.0)) {
      throw std::invalid_argument("data_timeout must be positive");
    }
    rclcpp::SubscriptionOptions options;
    options.callback_group = callback_group_;
    sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic, rclcpp::QoS(1).reliable().durability_volatile(),
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          // 不接受错误尺寸/旋转栅格。适配节点只输出轴对齐的 odom 栅格。
          const auto &o = msg->info.origin;
          if (msg->header.frame_id.empty() ||
              !std::isfinite(msg->info.resolution) ||
              msg->info.resolution <= 0 || msg->info.width == 0 ||
              msg->info.height == 0 ||
              msg->data.size() !=
                  static_cast<size_t>(msg->info.width) * msg->info.height ||
              !std::isfinite(o.position.x) || !std::isfinite(o.position.y) ||
              !std::isfinite(o.orientation.w) ||
              std::abs(std::abs(o.orientation.w) - 1.0) > 1e-6 ||
              o.orientation.x != 0.0 || o.orientation.y != 0.0 ||
              o.orientation.z != 0.0 ||
              std::any_of(msg->data.begin(), msg->data.end(),
                          [](int8_t v) { return v < -1 || v > 100; })) {
            RCLCPP_ERROR_THROTTLE(logger_, *clock_, 2000,
                                  "拒绝格式异常的地形栅格");
            return;
          }
          std::lock_guard<std::mutex> lock(mutex_);
          latest_ = msg;
          received_ = std::chrono::steady_clock::now();
        },
        options);
    current_ = false;
  }

  void reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_.reset();
    current_ = false;
  }
  // 清地图恢复动作不能把真实障碍清成可通行；障碍消失由 ROGMap 射线与衰减决定。
  bool isClearable() override { return false; }
  
  void updateBounds(double, double, double, double *min_x, double *min_y,
                    double *max_x, double *max_y) override {
    working_.reset();
    // 包含上一帧区域，主代价地图才能擦掉已移动/消失障碍；不能只更新新窗口。
    if (have_previous_) {
      expand(previous_, min_x, min_y, max_x, max_y);
    }
    have_previous_ = false;
    if (!enabled_) {
      current_ = true;
      return;
    }
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
    double receive_age;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      grid = latest_;
      receive_age = std::chrono::duration<double>(
                        std::chrono::steady_clock::now() - received_)
                        .count();
    }
    current_ = false;
    if (!grid) {
      return;
    }
    const double data_age =
        (clock_->now() - rclcpp::Time(grid->header.stamp)).seconds();
    if (data_age > timeout_ || data_age < -0.1 || receive_age > timeout_) {
      RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 2000,
          "地形数据过期：消息年龄 %.3f s，接收间隔 %.3f s；costmap 标为非实时",
          data_age, receive_age);
      return;
    }
    tx_ = ty_ = yaw_ = 0.0;
    if (grid->header.frame_id != layered_costmap_->getGlobalFrameID()) {
      try {
        // 使用地形快照时间对应的 map<-odom；SLAM 回环修正后，
        // 用最新 TF 投影旧快照会让局部障碍相对 /map 瞬间错位。
        const auto transform =
            tf_->lookupTransform(layered_costmap_->getGlobalFrameID(),
                                 grid->header.frame_id,
                                 rclcpp::Time(grid->header.stamp));
        const auto &q = transform.transform.rotation;
        tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
        if (!std::isfinite(rotation.length2()) || rotation.length2() < 1e-12) {
          return;
        }
        rotation.normalize();
        double roll, pitch;
        tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw_);
        tx_ = transform.transform.translation.x;
        ty_ = transform.transform.translation.y;
        const rclcpp::Time tf_stamp(transform.header.stamp);
        const double tf_age = (clock_->now() - tf_stamp).seconds();
        if (!std::isfinite(tx_) || !std::isfinite(ty_) ||
            !std::isfinite(yaw_) || std::abs(roll) > 0.1 ||
            std::abs(pitch) > 0.1 ||
            (tf_stamp.nanoseconds() != 0 &&
             (tf_age > timeout_ || tf_age < -timeout_))) {
          RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000,
                               "地形 TF 过期或非平面变换；拒绝错误投影");
          return;
        }
      } catch (const tf2::TransformException &error) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "地形 TF 不可用：%s",
                             error.what());
        return;
      }
    }
    c_ = std::cos(yaw_);
    s_ = std::sin(yaw_);
    const double x0 = grid->info.origin.position.x,
                 y0 = grid->info.origin.position.y;
    const double x1 = x0 + grid->info.width * grid->info.resolution;
    const double y1 = y0 + grid->info.height * grid->info.resolution;
    previous_ = {std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity(),
                 -std::numeric_limits<double>::infinity()};
    for (const double x : {x0, x1}) {
      for (const double y : {y0, y1}) {
        const double gx = c_ * x - s_ * y + tx_, gy = s_ * x + c_ * y + ty_;
        previous_[0] = std::min(previous_[0], gx);
        previous_[1] = std::min(previous_[1], gy);
        previous_[2] = std::max(previous_[2], gx);
        previous_[3] = std::max(previous_[3], gy);
      }
    }
    expand(previous_, min_x, min_y, max_x, max_y);
    have_previous_ = true;
    working_ = grid;
    current_ = true;
  }

  void updateCosts(nav2_costmap_2d::Costmap2D &master, int min_i, int min_j,
                   int max_i, int max_j) override {
    if (!enabled_ || !working_) {
      return;
    }
    const auto &grid = *working_;
    const double resolution = grid.info.resolution;
    // 保守重采样：主地图的格子可能更粗或相对旋转，不允许细墙恰好落在采样点之间消失。
    const double half_extent =
        0.5 * master.getResolution() * (std::abs(c_) + std::abs(s_));
    for (int j = min_j; j < max_j; ++j) {
      for (int i = min_i; i < max_i; ++i) {
        double wx, wy;
        master.mapToWorld(i, j, wx, wy);
        const double x = c_ * (wx - tx_) + s_ * (wy - ty_);
        const double y = -s_ * (wx - tx_) + c_ * (wy - ty_);
        const int x0 =
            cellIndex(x - half_extent, grid.info.origin.position.x, resolution);
        const int y0 =
            cellIndex(y - half_extent, grid.info.origin.position.y, resolution);
        const int x1 = cellIndex(x + half_extent - 1e-9,
                                 grid.info.origin.position.x, resolution);
        const int y1 = cellIndex(y + half_extent - 1e-9,
                                 grid.info.origin.position.y, resolution);
        int occupancy = -1;
        bool unknown = x0 < 0 || y0 < 0 ||
                       x1 >= static_cast<int>(grid.info.width) ||
                       y1 >= static_cast<int>(grid.info.height);
        for (int sy = std::max(0, y0);
             sy <= std::min(y1, static_cast<int>(grid.info.height) - 1); ++sy) {
          for (int sx = std::max(0, x0);
               sx <= std::min(x1, static_cast<int>(grid.info.width) - 1);
               ++sx) {
            const auto value =
                grid.data[static_cast<size_t>(sy) * grid.info.width + sx];
            unknown = unknown || value < 0;
            occupancy = std::max(occupancy, static_cast<int>(value));
          }
        }
        if (occupancy < 0 || (occupancy == 0 && unknown)) {
          continue;
        }
        const auto cost = toCost(static_cast<int8_t>(occupancy));
        const auto old = master.getCost(i, j);
        // 实时自由空间不能覆盖静态地图里的墙；已观测自由空间可填充未知区域。
        if (old == 255 || cost > old) {
          master.setCost(i, j, cost);
        }
      }
    }
  }

private:
  static void expand(const std::array<double, 4> &b, double *x0, double *y0,
                     double *x1, double *y1) {
    *x0 = std::min(*x0, b[0]);
    *y0 = std::min(*y0, b[1]);
    *x1 = std::max(*x1, b[2]);
    *y1 = std::max(*y1, b[3]);
  }
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr latest_, working_;
  std::chrono::steady_clock::time_point received_{};
  std::array<double, 4> previous_{};
  bool have_previous_{false};
  double timeout_{0.7}, tx_{0.0}, ty_{0.0}, yaw_{0.0}, c_{1.0}, s_{0.0};
};
} // namespace sentry_terrain_bridge
PLUGINLIB_EXPORT_CLASS(sentry_terrain_bridge::TerrainLayer,
                       nav2_costmap_2d::Layer)
