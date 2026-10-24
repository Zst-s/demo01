#pragma once

#include <algorithm>
#include <cmath>
#include <vector>
#include <tf2/LinearMath/Transform.h>

namespace point_lio_frames {
// One scan only: never bridge a reset or an unobserved inter-scan interval.
class DenseTerrainTrajectory {
 public:
  void clear() { samples_.clear(); }

  bool append(double time, const tf2::Transform &pose) {
    const auto q = pose.getRotation();
    const auto t = pose.getOrigin();
    if (!std::isfinite(time) || !std::isfinite(t.x()) ||
        !std::isfinite(t.y()) || !std::isfinite(t.z()) ||
        !std::isfinite(q.length2()) || std::abs(q.length2() - 1.) > 1e-6) {
      clear();
      return false;
    }
    if (!samples_.empty() && time < samples_.back().time) clear();
    if (!samples_.empty() && time == samples_.back().time) samples_.pop_back();
    samples_.push_back({time, pose});
    return true;
  }

  bool poseAt(double time, tf2::Transform &pose) const {
    if (!std::isfinite(time) || samples_.empty() ||
        time < samples_.front().time || time > samples_.back().time) return false;
    const auto right = std::lower_bound(samples_.begin(), samples_.end(), time,
        [](const Sample &s, double t) { return s.time < t; });
    if (right->time == time) { pose = right->pose; return true; }
    const auto &left = *(right - 1);
    const double interval = right->time - left.time;
    // Missing trajectory coverage is not permission to extrapolate a cloud.
    if (interval > 0.02) return false;
    const double alpha = (time - left.time) / interval;
    pose.setOrigin(left.pose.getOrigin().lerp(right->pose.getOrigin(), alpha));
    pose.setRotation(left.pose.getRotation().slerp(right->pose.getRotation(), alpha));
    return true;
  }

 private:
  struct Sample { double time; tf2::Transform pose; };
  std::vector<Sample> samples_;
};
}  // namespace point_lio_frames
