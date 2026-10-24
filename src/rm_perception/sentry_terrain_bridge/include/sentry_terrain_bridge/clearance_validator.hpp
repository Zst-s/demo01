#pragma once

// 只读、体素级直线通道诊断。绝不输出导航许可，也不把二维 PASSABLE 当作净空。
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace sentry_terrain_bridge {

enum class ObservedState { UNKNOWN, FREE, OCCUPIED, OUTSIDE };
struct ObservedVoxel {
  ObservedState state{ObservedState::UNKNOWN};
  double direct_stamp{0.0};  // 直接射线观测时间；0 表示无证据，衰减不得刷新此值。
};

struct ClearanceConfig {
  double robot_width{0.0}, robot_length{0.0}, robot_height{0.0};
  double horizontal_margin{0.0}, vertical_margin{0.0};
  double max_observation_age{1.0};
  double entry_recent_age{2.0};
  int confirmations{3};
  size_t max_voxel_checks{200000};
};

struct PassageProbe {
  double x{0.0}, y{0.0}, ground_z{0.0}, yaw{0.0};
  double travel_length{0.0};  // 车体中心沿直线移动的距离，不是车长。
};

struct ClearanceResult {
  struct Cell { double x, y, z; };
  // 支撑缺证据不等于已观测到悬空；单独保留其原因。
  size_t body_unknown{0}, support_unknown{0}, support_free{0};
  size_t support_stale{0}, support_outside{0};
  std::vector<Cell> body_unknown_cells, support_missing_cells, body_blocked_cells;
  bool geometry_pass{false};
  std::string reason{"INVALID_CONFIGURATION"};
  size_t footprint_cells{0}, checked{0}, unknown{0}, stale{0}, outside{0};
  size_t blocked{0}, unsupported{0};
  size_t body_voxels{0}, recent_body_voxels{0};
  // Four travel-axis slices × two height bands: diagnose occluded body voxels.
  std::array<size_t, 8> body_by_slice{};
  std::array<size_t, 8> recent_body_by_slice{};
  // Ground-support recency by the same four travel-axis slices. Entry logic
  // must inspect the advancing edge, not require a fresh scan of the whole
  // tunnel (which is often occluded from a single real sensor viewpoint).
  std::array<size_t, 4> support_by_slice{};
  std::array<size_t, 4> recent_support_by_slice{};
  size_t entry_edge_voxels{0}, recent_entry_edge_voxels{0};
  double oldest_entry_observation{std::numeric_limits<double>::infinity()};
  double newest_entry_observation{0.0};
  double oldest_observation{std::numeric_limits<double>::infinity()};
  double newest_observation{0.0};
  double oldest_body_observation{std::numeric_limits<double>::infinity()};
  double newest_body_observation{0.0};
  double oldest_support_observation{std::numeric_limits<double>::infinity()};
  double newest_support_observation{0.0};
  double bad_x{0.0}, bad_y{0.0}, bad_z{0.0};
  bool has_bad_cell{false};
};

inline bool validClearanceConfig(const ClearanceConfig & c) {
  return std::isfinite(c.robot_width) && c.robot_width > 0.0 &&
    std::isfinite(c.robot_length) && c.robot_length > 0.0 &&
    std::isfinite(c.robot_height) && c.robot_height > 0.0 &&
    std::isfinite(c.horizontal_margin) && c.horizontal_margin >= 0.0 &&
    std::isfinite(c.vertical_margin) && c.vertical_margin >= 0.0 &&
    std::isfinite(c.max_observation_age) && c.max_observation_age > 0.0 &&
    std::isfinite(c.entry_recent_age) && c.entry_recent_age > 0.0 &&
    c.entry_recent_age <= 5.0 &&
    c.confirmations >= 2 && c.max_voxel_checks > 0 && c.max_voxel_checks <= 1000000;
}

// ORIGIN_AT_CORNER：全局索引 i 对应中心 (i+0.5)*resolution。
// 检查整个车体平移扫掠矩形，不只检查中心线；本版不覆盖转弯/坡道/台阶。
inline ClearanceResult validateClearance(
  const ClearanceConfig & c, const PassageProbe & p, double resolution, double now,
  const std::function<ObservedVoxel(int, int, int)> & query,
  const std::function<double(int, int)> & support_height = {})
{
  ClearanceResult result;
  if (!validClearanceConfig(c) || !query || !std::isfinite(resolution) || resolution <= 0.0 ||
      !std::isfinite(now) || now <= 0.0 || !std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.ground_z) || !std::isfinite(p.yaw) ||
      !std::isfinite(p.travel_length) || p.travel_length <= 0.0) { return result; }
  const double co = std::cos(p.yaw), si = std::sin(p.yaw);
  const double hl = (p.travel_length + c.robot_length) * 0.5 + c.horizontal_margin;
  const double hw = c.robot_width * 0.5 + c.horizontal_margin;
  const double ex = std::abs(co) * hl + std::abs(si) * hw;
  const double ey = std::abs(si) * hl + std::abs(co) * hw;
  const double lo_x = std::floor((p.x - ex) / resolution);
  const double hi_x = std::floor((p.x + ex) / resolution);
  const double lo_y = std::floor((p.y - ey) / resolution);
  const double hi_y = std::floor((p.y + ey) / resolution);
  const double floor_id = std::floor(p.ground_z / resolution);
  // 地面命中体素内高度不确定，向上额外保留一层体素余量。
  const double body_layers = std::ceil((c.robot_height + c.vertical_margin) / resolution) + 1;
  for (double value : {lo_x, hi_x, lo_y, hi_y, floor_id, body_layers}) {
    if (!std::isfinite(value) || std::abs(value) > 10000000) {
      result.reason = "QUERY_BUDGET_EXCEEDED"; return result;
    }
  }
  const double budget = (hi_x - lo_x + 1) * (hi_y - lo_y + 1) * (body_layers + 1);
  if (budget > static_cast<double>(c.max_voxel_checks)) {
    result.reason = "QUERY_BUDGET_EXCEEDED"; return result;
  }
  const double half_cell = resolution * 0.5;
  const double projected_half_cell = half_cell * (std::abs(co) + std::abs(si));
  for (int y = static_cast<int>(lo_y); y <= static_cast<int>(hi_y); ++y) {
    for (int x = static_cast<int>(lo_x); x <= static_cast<int>(hi_x); ++x) {
      const double wx = (x + 0.5) * resolution, wy = (y + 0.5) * resolution;
      const double dx = wx - p.x, dy = wy - p.y;
      // 矩形与体素正方形的分离轴判定；边界相交也纳入检查，避免漏掉窄障碍。
      if (std::abs(dx) > ex + half_cell || std::abs(dy) > ey + half_cell ||
          std::abs(co * dx + si * dy) > hl + projected_half_cell ||
          std::abs(-si * dx + co * dy) > hw + projected_half_cell) { continue; }
      ++result.footprint_cells;
      const double local_floor = support_height ? support_height(x, y) : p.ground_z;
      if (!std::isfinite(local_floor) || std::abs(local_floor / resolution) > 10000000) {
        result.geometry_pass = false;
        result.reason = "INVALID_SUPPORT_MODEL";
        return result;
      }
      const int gz = static_cast<int>(std::floor(local_floor / resolution));
      const double along = co * dx + si * dy;
      const int axial = std::clamp(static_cast<int>(
        std::floor((along + hl) * 4.0 / (2.0 * hl))), 0, 3);
      for (int z = gz; z <= gz + static_cast<int>(body_layers); ++z) {
        const auto voxel = query(x, y, z);
        ++result.checked;
        int slice = -1;
        if (z > gz) {
          ++result.body_voxels;
          const int height = (z - gz - 1) * 2 / static_cast<int>(body_layers);
          slice = axial * 2 + std::min(height, 1);
          ++result.body_by_slice[slice];
        }
        bool bad = false;
        if (voxel.state == ObservedState::OUTSIDE) { ++result.outside; bad = true; }
        else if (voxel.state == ObservedState::UNKNOWN) { ++result.unknown; bad = true; }
        else {
          const double age = now - voxel.direct_stamp;
          if (!std::isfinite(voxel.direct_stamp) || voxel.direct_stamp <= 0.0 ||
              age < 0.0 || age > c.max_observation_age) { ++result.stale; bad = true; }
          else {
            if (axial == 3) {
              ++result.entry_edge_voxels;
              if (age <= c.entry_recent_age) { ++result.recent_entry_edge_voxels; }
              result.oldest_entry_observation = std::min(
                result.oldest_entry_observation, voxel.direct_stamp);
              result.newest_entry_observation = std::max(
                result.newest_entry_observation, voxel.direct_stamp);
            }
            result.oldest_observation = std::min(result.oldest_observation, voxel.direct_stamp);
            result.newest_observation = std::max(result.newest_observation, voxel.direct_stamp);
          if (z > gz) {
              if (age <= c.entry_recent_age) {
                ++result.recent_body_voxels;
                ++result.recent_body_by_slice[slice];
              }
              result.oldest_body_observation = std::min(
                result.oldest_body_observation, voxel.direct_stamp);
              result.newest_body_observation = std::max(
                result.newest_body_observation, voxel.direct_stamp);
          } else {
            ++result.support_by_slice[axial];
            if (age <= c.entry_recent_age) {
              ++result.recent_support_by_slice[axial];
            }
            result.oldest_support_observation = std::min(
                result.oldest_support_observation, voxel.direct_stamp);
              result.newest_support_observation = std::max(
                result.newest_support_observation, voxel.direct_stamp);
            }
          }
          if (z == gz && voxel.state != ObservedState::OCCUPIED) {
            ++result.unsupported; bad = true;
          }
          if (z > gz && voxel.state == ObservedState::OCCUPIED) {
            ++result.blocked; bad = true;
            result.body_blocked_cells.push_back({wx, wy, (z + 0.5) * resolution});
          }
        }
        if (bad && !result.has_bad_cell) {
          result.has_bad_cell = true;
          result.bad_x = wx; result.bad_y = wy; result.bad_z = (z + 0.5) * resolution;
        }
        if (z > gz && voxel.state == ObservedState::UNKNOWN) {
          ++result.body_unknown;
          result.body_unknown_cells.push_back({wx, wy, (z + 0.5) * resolution});
        }
        if (z == gz && bad) {
          result.support_missing_cells.push_back({wx, wy, (z + 0.5) * resolution});
          if (voxel.state == ObservedState::UNKNOWN) { ++result.support_unknown; }
          else if (voxel.state == ObservedState::OUTSIDE) { ++result.support_outside; }
          else if (voxel.state == ObservedState::FREE) { ++result.support_free; }
          else { ++result.support_stale; }
        }
      }
    }
  }
  // 优先报告硬障碍；计数仍保留所有失败类别，避免用单一标签掩盖其他问题。
  if (result.blocked) { result.reason = "BODY_VOLUME_OCCUPIED"; }
  else if (result.unsupported) { result.reason = "NO_GROUND_SUPPORT"; }
  else if (result.outside) { result.reason = "OUTSIDE_ROLLING_MAP"; }
  else if (result.unknown) { result.reason = "UNKNOWN_VOLUME_OR_SUPPORT"; }
  else if (result.stale) { result.reason = "STALE_OR_UNMEASURED"; }
  else if (result.footprint_cells && std::isfinite(result.oldest_observation)) {
    result.geometry_pass = true; result.reason = "GEOMETRY_PASS_OBSERVATION_ONLY";
  }
  return result;
}

// 多次读取同一份体素不能冒充多帧确认；每次须有新的直接观测命中被查体积。
// 全体积的新鲜度已经由 validateClearance 独立检查，不要求每帧重新照到所有体素。
class ClearanceConfirmation {
public:
  void reset() { count_ = 0; newest_ = 0.0; last_now_ = 0.0; revision_ = 0; }
  int update(const ClearanceResult & result, uint64_t revision, double now,
             const ClearanceConfig & config) {
    if (revision != revision_ || now < last_now_ ||
        (last_now_ > 0 && now - last_now_ > config.max_observation_age)) { reset(); }
    revision_ = revision; last_now_ = now;
    if (!result.geometry_pass || !std::isfinite(result.oldest_observation) ||
        !std::isfinite(result.newest_observation) ||
        !std::isfinite(now) || now < result.oldest_observation ||
        now - result.oldest_observation > config.max_observation_age) {
      count_ = 0; newest_ = 0.0; return count_;
    }
    if (result.newest_observation > newest_) {
      count_ = std::min(count_ + 1, config.confirmations);
      newest_ = result.newest_observation;
    }
    return count_;
  }
private:
  int count_{0};
  double newest_{0.0}, last_now_{0.0};
  uint64_t revision_{0};
};

}  // namespace sentry_terrain_bridge
