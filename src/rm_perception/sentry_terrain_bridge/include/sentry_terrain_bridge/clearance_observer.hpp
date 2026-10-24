#pragma once

#include <iomanip>
#include <sstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rog_map_ros/rog_map_ros2.hpp>
#include <sentry_terrain_bridge/clearance_validator.hpp>
#include <sentry_terrain_bridge/support_surface.hpp>
#include <sentry_terrain_bridge/supported_clearance.hpp>

namespace sentry_terrain_bridge {
// 可选、只读净空观测；输出的 verified_segment 只是短段观测证据，不是通行许可。
// 与 TerrainBridge 同一单线程执行器，禁止并发读取正在滑动的地图。
class ClearanceObserver {
public:
  ClearanceObserver(const rclcpp::Node::SharedPtr & node, rog_map::ROGMapROS & map,
                    const std::string & frame)
  : node_(node), map_(map), frame_(frame) {
    config_.robot_length = node_->declare_parameter<double>("clearance.robot_length", 0.0);
    config_.robot_width = node_->declare_parameter<double>("clearance.robot_width", 0.0);
    config_.robot_height = node_->declare_parameter<double>("clearance.robot_height", 0.0);
    config_.max_observation_age = node_->declare_parameter<double>("clearance.max_observation_age", 1.0);
    config_.entry_recent_age = node_->declare_parameter<double>("clearance.entry_recent_age", 2.0);
    config_.confirmations = node_->declare_parameter<int>("clearance.confirmations", 3);
    travel_length_ = node_->declare_parameter<double>("clearance.travel_length", 1.0);
    support_diagnostic_ = node_->declare_parameter<bool>("clearance.support_diagnostic", false);
    support_mode_ = node_->declare_parameter<std::string>("clearance.support_mode", "fixed");
    support_max_slope_ = node_->declare_parameter<double>("clearance.support_max_slope", 0.35);
    if ((support_mode_ != "fixed" && support_mode_ != "shadow") ||
        !std::isfinite(support_max_slope_) || support_max_slope_<0 || support_max_slope_>.6) {
      throw std::invalid_argument("clearance.support_mode: fixed or shadow; slope in [0,0.6] radians");
    }
    if (!validClearanceConfig(config_) || !std::isfinite(travel_length_) || travel_length_ <= 0) {
      throw std::invalid_argument("clearance: 必须明确配置有效尺寸和正的查询长度");
    }
    map_.enableObservationTracking();
    status_ = node_->create_publisher<std_msgs::msg::String>("/terrain/clearance/status", 1);
    virtual_preview_ = node_->declare_parameter<bool>("clearance.virtual_preview", false);
    publish_verified_segments_ = node_->declare_parameter<bool>(
      "clearance.publish_verified_segments", false);
    if (virtual_preview_ && publish_verified_segments_) {
      throw std::invalid_argument("virtual clearance preview must never publish verified passage evidence");
    }
    if (publish_verified_segments_ || virtual_preview_) {
      segment_pub_ = node_->create_publisher<std_msgs::msg::String>(
        virtual_preview_ ? "/terrain/clearance/preview_segment" :
                           "/terrain/clearance/verified_segment", 10);
    }
    markers_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("/terrain/clearance/markers", 1);
    probe_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/terrain/clearance/probe", 1,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        const auto & q = msg->pose.orientation;
        const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
        const double age = (node_->now() - rclcpp::Time(msg->header.stamp)).seconds();
        if (msg->header.frame_id != frame_ || !std::isfinite(norm) ||
            std::abs(norm - 1.0) > 0.01 || std::abs(q.x) > 0.01 || std::abs(q.y) > 0.01 ||
            age < -0.1 || age > 1.0) {
          have_probe_ = false; confirmation_.reset(); last_published_observation_ = 0.0;
          RCLCPP_WARN(node_->get_logger(), "拒绝探针：坐标系、时间戳或四元数无效");
          return;
        }
        probe_.x = msg->pose.position.x; probe_.y = msg->pose.position.y;
        probe_.ground_z = msg->pose.position.z;
        probe_.yaw = std::atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z));
        probe_.travel_length = travel_length_;
        have_probe_ = std::isfinite(probe_.x) && std::isfinite(probe_.y) && std::isfinite(probe_.ground_z);
        if (!have_probe_) { probe_ = PassageProbe{}; }
        ++revision_; confirmation_.reset(); last_published_observation_ = 0.0;
      });
    timer_ = node_->create_wall_timer(std::chrono::milliseconds(200), [this]() { tick(); });
  }
  void inputUpdated(double stamp) { input_stamp_ = stamp; }

private:
  void tick() {
    const double now = node_->now().seconds();
    ClearanceResult result;
    result.reason = "NO_PROBE";
    if (have_probe_) {
      if (input_stamp_ <= 0 || now < input_stamp_ || now-input_stamp_ > config_.max_observation_age) {
        result.reason = "STALE_INPUT";
      } else {
        result = validateClearance(config_, probe_, map_.getResolution(), now,
          [this](int x, int y, int z) {
            const super_utils::Vec3i id(x, y, z);
            ObservedVoxel v;
            const auto type = map_.rawObservedType(id);
            if (type == super_utils::GridType::OCCUPIED) { v.state = ObservedState::OCCUPIED; }
            else if (type == super_utils::GridType::KNOWN_FREE) { v.state = ObservedState::FREE; }
            else if (type == super_utils::GridType::OUT_OF_MAP) { v.state = ObservedState::OUTSIDE; }
            v.direct_stamp = map_.cellDirectObservationTime(id);
            return v;
          });
      }
    }
    const int count = confirmation_.update(result, revision_, now, config_);
    SupportSurfaceResult support_result;
    ClearanceResult relocated_result;
    SupportedClearance supported;
    if(support_mode_=="shadow" && have_probe_ && input_stamp_>0 &&
       now>=input_stamp_ && now-input_stamp_<=config_.max_observation_age) {
      supported=validateSupportedClearance(config_,probe_,map_.getResolution(),now,
        [this](int x,int y,int z) {
          const super_utils::Vec3i id(x,y,z);
          ObservedVoxel v;const auto type=map_.rawObservedType(id);
          if(type==super_utils::GridType::OCCUPIED)v.state=ObservedState::OCCUPIED;
          else if(type==super_utils::GridType::KNOWN_FREE)v.state=ObservedState::FREE;
          else if(type==super_utils::GridType::OUT_OF_MAP)v.state=ObservedState::OUTSIDE;
          v.direct_stamp=map_.cellDirectObservationTime(id);return v;
        },support_max_slope_);
    }
    if (support_diagnostic_ && have_probe_ && input_stamp_>0 &&
        now>=input_stamp_ && now-input_stamp_<=config_.max_observation_age) {
      auto query = [this](int x,int y,int z) {
        const super_utils::Vec3i id(x,y,z);
        ObservedVoxel v; const auto type=map_.rawObservedType(id);
        if(type==super_utils::GridType::OCCUPIED)v.state=ObservedState::OCCUPIED;
        else if(type==super_utils::GridType::KNOWN_FREE)v.state=ObservedState::FREE;
        else if(type==super_utils::GridType::OUT_OF_MAP)v.state=ObservedState::OUTSIDE;
        v.direct_stamp=map_.cellDirectObservationTime(id);return v;
      };
      support_result=estimateSupportSurface(config_,probe_,map_.getResolution(),now,query);
      if(support_result.complete) {
        auto candidate=probe_;candidate.ground_z=support_result.ground_z;
        relocated_result=validateClearance(config_,candidate,map_.getResolution(),now,query);
      }
      // Deliberately does not assign probe_, result, count or publish any evidence.
    }
    std::ostringstream text;
    text << (virtual_preview_ ? "VIRTUAL_BODY_PREVIEW_ONLY" : "OBSERVATION_ONLY")
         << std::setprecision(17) << " stamp=" << now << " frame=" << frame_
         << " robot_height=" << config_.robot_height
         << " probe_x=" << probe_.x << " probe_y=" << probe_.y
         << " ground_z=" << probe_.ground_z << " probe_yaw=" << probe_.yaw
         << " support_mode=" << support_mode_
         << " measured_model=" << supported.model_reason
         << " measured_ground_z=" << supported.ground_z
         << " measured_slope=" << supported.slope
         << " measured_segmented=" << supported.segmented
         << " measured_break_along=" << supported.break_along
         << " measured_slope_after=" << supported.slope_after
         << " measured_model_outliers=" << supported.model_outliers.size()
         << " measured_max_residual=" << supported.max_residual
         << " measured_approach_residual=" << supported.approach_max_residual
         << " measured_reason=" << supported.clearance.reason
         << " measured_evaluated=" << supported.body_evaluated
         << " measured_blocked=" << (supported.body_evaluated ? static_cast<long long>(supported.clearance.blocked) : -1)
         << " measured_body_unknown=" << (supported.body_evaluated ? static_cast<long long>(supported.clearance.body_unknown) : -1)
         << " measured_support_missing=" << (supported.body_evaluated ? static_cast<long long>(supported.clearance.support_missing_cells.size()) : -1)
         << " measured_queries=" << supported.queries
         << " travel_length=" << travel_length_ << " resolution=" << map_.getResolution()
         << " revision=" << revision_ << " input_age=" << (now-input_stamp_)
         << " reason=" << result.reason << " confirmed=" << count << '/' << config_.confirmations
         << " checked=" << result.checked << " blocked=" << result.blocked
         << " unknown=" << result.unknown << " outside=" << result.outside
         << " stale=" << result.stale << " unsupported=" << result.unsupported
         << " body_unknown=" << result.body_unknown
         << " body_recent=" << result.recent_body_voxels << '/' << result.body_voxels
         << " support_missing=" << result.support_missing_cells.size()
         << " support_unknown=" << result.support_unknown
         << " support_free=" << result.support_free
         << " support_stale=" << result.support_stale
         << " support_outside=" << result.support_outside
         << " support_oldest_age=" << (now-result.oldest_support_observation)
         << " support_newest_age=" << (now-result.newest_support_observation)
         << " entry_edge_recent=" << result.recent_entry_edge_voxels
         << '/' << result.entry_edge_voxels
         << " entry_edge_oldest_age=" << (now-result.oldest_entry_observation)
         << " entry_edge_newest_age=" << (now-result.newest_entry_observation)
         << " body_recent_by_slice=";
    for (size_t i = 0; i < result.body_by_slice.size(); ++i) {
      if (i) { text << ','; }
      text << result.recent_body_by_slice[i] << '/' << result.body_by_slice[i];
    }
    text
         << " support_recent_by_slice=";
    for (size_t i = 0; i < result.support_by_slice.size(); ++i) {
      if (i) { text << ','; }
      text << result.recent_support_by_slice[i] << '/' << result.support_by_slice[i];
    }
    if(support_diagnostic_) {
      text << " support_diagnostic=" << support_result.reason
           << " support_candidate_z=" << support_result.ground_z
           << " support_columns=" << support_result.columns
           << " support_unobserved_columns=" << support_result.missing
           << " support_ambiguous_columns=" << support_result.ambiguous
           << " support_discontinuous_columns=" << support_result.discontinuous
           << " support_profile_rows=" << support_result.rows.size()
           << " support_max_adjacent_row_delta=" << support_result.max_observed_row_delta
           << " support_max_lateral_row_spread=" << support_result.max_row_spread
           << " relocated_diagnostic=" << (support_result.complete ? relocated_result.reason : "NOT_EVALUATED");
    }
    std_msgs::msg::String status; status.data = text.str(); status_->publish(status);
    if (segment_pub_ && count >= config_.confirmations && result.geometry_pass &&
        result.newest_observation > last_published_observation_) {
      // 结构记忆/入口门控必须另外核验 map 关联、时间、障碍与行驶边界。
      // 这里不接收二维 PASSABLE，也绝不直接清除 Costmap。
      std::ostringstream evidence;
      evidence << std::setprecision(17)
               << "{\"schema\":1,\"kind\":\""
               << (virtual_preview_ ? "virtual_short_segment_preview" :
                                      "verified_short_segment_observation") << "\""
               << ",\"frame\":\"" << frame_ << "\""
               << ",\"probe_x\":" << probe_.x << ",\"probe_y\":" << probe_.y
               << ",\"ground_z\":" << probe_.ground_z << ",\"yaw\":" << probe_.yaw
               << ",\"travel_length\":" << probe_.travel_length
               << ",\"robot_length\":" << config_.robot_length
               << ",\"robot_width\":" << config_.robot_width
               << ",\"robot_height\":" << config_.robot_height
               << ",\"resolution\":" << map_.getResolution()
               << ",\"revision\":" << revision_
               << ",\"confirmed_frames\":" << count
               << ",\"measured_at\":" << now
               << ",\"oldest_observation\":" << result.oldest_observation
               << ",\"newest_observation\":" << result.newest_observation
               << ",\"oldest_body_observation\":" << result.oldest_body_observation
               << ",\"newest_body_observation\":" << result.newest_body_observation
               << ",\"oldest_support_observation\":" << result.oldest_support_observation
               << ",\"newest_support_observation\":" << result.newest_support_observation
               << ",\"oldest_entry_observation\":" << result.oldest_entry_observation
               << ",\"newest_entry_observation\":" << result.newest_entry_observation
               << ",\"recent_entry_edge_voxels\":" << result.recent_entry_edge_voxels
               << ",\"entry_edge_voxels\":" << result.entry_edge_voxels
               << ",\"recent_body_voxels\":" << result.recent_body_voxels
               << ",\"body_voxels\":" << result.body_voxels
               << ",\"entry_recent_window\":" << config_.entry_recent_age
               << ",\"recent_support_by_slice\":[";
      for (size_t i = 0; i < result.support_by_slice.size(); ++i) {
        if (i) { evidence << ','; }
        evidence << '[' << result.recent_support_by_slice[i] << ','
                 << result.support_by_slice[i] << ']';
      }
      evidence << ']'
               << ",\"recent_body_by_slice\":[";
      for (size_t i = 0; i < result.body_by_slice.size(); ++i) {
        if (i) { evidence << ','; }
        evidence << '[' << result.recent_body_by_slice[i] << ','
                 << result.body_by_slice[i] << ']';
      }
      evidence << ']'
               << ",\"valid_until\":" << std::min(
                    result.oldest_observation + config_.max_observation_age,
                    now + std::min(config_.max_observation_age, 1.0))
               << ",\"checked_voxels\":" << result.checked << '}';
      std_msgs::msg::String message; message.data = evidence.str();
      segment_pub_->publish(message);
      last_published_observation_ = result.newest_observation;
    }
    visualization_msgs::msg::MarkerArray array;
    using Marker = visualization_msgs::msg::Marker;
    Marker box;
    box.header.frame_id = frame_; box.header.stamp = node_->now();
    box.ns = "clearance_probe"; box.id = 0; box.type = Marker::CUBE;
    box.action = have_probe_ ? Marker::ADD : Marker::DELETE;
    box.pose.position.x = probe_.x; box.pose.position.y = probe_.y;
    // Show the actual voxel query volume, including its upper safety layer,
    // rather than a smaller ideal body box which hides blocking roof cells.
    const double resolution = map_.getResolution();
    const double support_bottom = std::floor(probe_.ground_z / resolution) * resolution;
    const double body_bottom = support_bottom + resolution;
    const double body_depth = (std::ceil(
      (config_.robot_height + config_.vertical_margin) / resolution) + 1) * resolution;
    box.pose.position.z = body_bottom + body_depth / 2;
    box.pose.orientation.w = std::cos(probe_.yaw / 2);
    box.pose.orientation.z = std::sin(probe_.yaw / 2);
    box.scale.x = travel_length_ + config_.robot_length + 2 * config_.horizontal_margin;
    box.scale.y = config_.robot_width + 2 * config_.horizontal_margin;
    box.scale.z = body_depth;
    box.color.r = count >= config_.confirmations ? 0.0F : 1.0F;
    box.color.g = count >= config_.confirmations ? 1.0F : 0.4F;
    box.color.a = 0.06F; box.lifetime.sec = 1;
    array.markers.push_back(box);
    Marker label = box;
    label.id = 1; label.type = Marker::TEXT_VIEW_FACING;
    label.pose.position.z = probe_.ground_z + config_.robot_height + 0.25;
    label.pose.orientation.z = 0; label.pose.orientation.w = 1;
    label.scale.z = 0.08; label.color.a = 1;
    label.text = "Body unknown: " + std::to_string(result.body_unknown) +
      "\nSupport missing: " + std::to_string(result.support_missing_cells.size()) +
      "\nBlocked: " + std::to_string(result.blocked) + " | Stale: " + std::to_string(result.stale) +
      (virtual_preview_ ? "\nVIRTUAL 0.32m BODY - NOT REAL VEHICLE" : "\nDIAGNOSTIC ONLY");
    array.markers.push_back(label);
    auto cells = [&](const std::string & ns, const std::vector<ClearanceResult::Cell> & points,
                     float r, float g, float b) {
      Marker m;
      m.header = box.header; m.ns = ns; m.id = 0; m.type = Marker::CUBE_LIST;
      m.action = points.empty() ? Marker::DELETE : Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = m.scale.y = m.scale.z = map_.getResolution() * 0.8;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.45;
      m.lifetime.sec = 1;
      for (const auto & cell : points) {
        geometry_msgs::msg::Point p; p.x = cell.x; p.y = cell.y; p.z = cell.z;
        m.points.push_back(p);
      }
      array.markers.push_back(m);
    };
    cells("body_unknown_cyan", result.body_unknown_cells, 0.0, 0.8, 1.0);
    cells("support_missing_magenta", result.support_missing_cells, 1.0, 0.0, 0.8);
    cells("body_blocked_red", result.body_blocked_cells, 1.0, 0.0, 0.0);
    cells("measured_body_blocked_shadow", supported.clearance.body_blocked_cells, 1.0, 0.3, 0.0);
    cells("measured_support_outliers_shadow", supported.model_outliers, 1.0, 1.0, 0.0);
    cells("measured_body_unknown_shadow", supported.clearance.body_unknown_cells, 0.0, 1.0, 1.0);
    cells("measured_support_missing_shadow", supported.clearance.support_missing_cells, 1.0, 0.0, 1.0);
    cells("support_candidates_yellow_diagnostic", support_result.candidates, 1.0, 0.8, 0.0);
    cells("support_unobserved_xy_diagnostic", support_result.unobserved_columns, 0.6, 0.0, 1.0);
    Marker profile;
    profile.header=box.header; profile.ns="support_profile_diagnostic";
    profile.id=0; profile.type=Marker::LINE_LIST;
    profile.action=support_result.rows.size()>1 ? Marker::ADD : Marker::DELETE;
    profile.pose.orientation.w=1.0;
    profile.scale.x=std::max(0.01,resolution*0.35);
    profile.color.r=1.0;profile.color.g=0.9;profile.color.b=0.0;profile.color.a=0.9;
    profile.lifetime.sec=1;
    for(size_t i=1;i<support_result.rows.size();++i) {
      const auto & previous=support_result.rows[i-1];
      const auto & current=support_result.rows[i];
      if(!previous.observed || !current.observed)continue;
      geometry_msgs::msg::Point start,end;
      start.x=previous.x;start.y=previous.y;start.z=previous.z+resolution;
      end.x=current.x;end.y=current.y;end.z=current.z+resolution;
      profile.points.push_back(start);profile.points.push_back(end);
    }
    if(profile.points.empty())profile.action=Marker::DELETE;
    array.markers.push_back(profile);
    Marker relocated = box;
    relocated.ns = "relocated_probe_diagnostic";
    relocated.action = support_result.complete ? Marker::ADD : Marker::DELETE;
    relocated.pose.position.z = (std::floor(support_result.ground_z/resolution)+1)*resolution + body_depth/2;
    relocated.color.r=0.0;relocated.color.g=0.5;relocated.color.b=1.0;
    array.markers.push_back(relocated);
    Marker support_label=label;
    support_label.ns="support_diagnostic_status";
    support_label.action=support_diagnostic_ && have_probe_ ? Marker::ADD : Marker::DELETE;
    support_label.pose.position.z+=.30;
    support_label.text="SUPPORT DIAGNOSTIC: " + support_result.reason +
      "\nProfile max step: " + std::to_string(support_result.max_observed_row_delta) + " m" +
      " | lateral spread: " + std::to_string(support_result.max_row_spread) + " m" +
      "\nUnobserved columns: " + std::to_string(support_result.missing) +
      "\nNOT USED FOR CLEARING OR ENTRY";
    array.markers.push_back(support_label);
    Marker support = box;
    support.ns = "support_query_plane";
    support.pose.position.z = support_bottom + resolution / 2;
    support.scale.z = resolution;
    support.color.r = 1.0; support.color.g = 0.0; support.color.b = 0.8;
    support.color.a = 0.15;
    array.markers.push_back(support);
    markers_->publish(array);
  }
  rclcpp::Node::SharedPtr node_;
  rog_map::ROGMapROS & map_;
  std::string frame_;
  ClearanceConfig config_;
  PassageProbe probe_;
  ClearanceConfirmation confirmation_;
  double travel_length_{1.0}, input_stamp_{0.0};
  double last_published_observation_{0.0};
  bool publish_verified_segments_{false};
  bool support_diagnostic_{false};
  std::string support_mode_{"fixed"};
  double support_max_slope_{0.35};
  bool virtual_preview_{false};
  bool have_probe_{false};
  uint64_t revision_{0};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr probe_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr segment_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace sentry_terrain_bridge
