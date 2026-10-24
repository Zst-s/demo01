#pragma once
#include <sentry_terrain_bridge/clearance_validator.hpp>

namespace sentry_terrain_bridge {
// Diagnostic only. Never changes the production probe or produces evidence.
struct SupportSurfaceResult {
  struct Row {
    double x{0.0}, y{0.0}, z{0.0};
    size_t observed{0}, missing{0}, ambiguous{0};
  };
  bool complete{false};
  std::string reason{"INVALID_CONFIGURATION"};
  double ground_z{0.0};
  double max_observed_row_delta{0.0}, max_row_spread{0.0};
  size_t missing{0}, ambiguous{0}, discontinuous{0}, columns{0};
  std::vector<ClearanceResult::Cell> candidates;
  std::vector<ClearanceResult::Cell> unobserved_columns;
  std::vector<Row> rows;
};

// Follow measured support from a strip BEFORE the body sweep into the passage.
// A candidate must have measured occupied support and measured free space above.
// Unknown is not free. Multiple decks are ambiguous, not automatically a floor.
// Only a nearly level plane is supported; steps/slopes do not authorize passage.
inline SupportSurfaceResult estimateSupportSurface(
  const ClearanceConfig & c, const PassageProbe & p, double r, double now,
  const std::function<ObservedVoxel(int,int,int)> & query,
  double search_below = .10, double search_above = .60)
{
  SupportSurfaceResult out;
  if (!validClearanceConfig(c) || !query || !std::isfinite(r) || r <= 0 ||
      !std::isfinite(now) || now <= 0 || !std::isfinite(p.x) ||
      !std::isfinite(p.y) || !std::isfinite(p.yaw) || !std::isfinite(p.ground_z) ||
      !std::isfinite(p.travel_length) || p.travel_length <= 0 ||
      !std::isfinite(search_below) || !std::isfinite(search_above) ||
      search_below < 0 || search_above < 0 || search_below+search_above > 2.) return out;
  const double hl=(p.travel_length+c.robot_length)/2+c.horizontal_margin;
  const double hw=c.robot_width/2+c.horizontal_margin;
  const double start=-hl-.30;
  const double n_along=std::ceil((hl-start)/r)+1;
  const double n_across=std::ceil(2*hw/r)+1;
  const double zlo=std::floor((p.ground_z-search_below)/r);
  const double zhi=std::floor((p.ground_z+search_above)/r);
  const int free_layers=std::max(2,static_cast<int>(std::ceil(.10/r)));
  if (!std::isfinite(n_along*n_across*(zhi-zlo+1)) ||
      n_along*n_across*(zhi-zlo+1)*(free_layers+1)>c.max_voxel_checks ||
      std::abs(zlo)>10000000 || std::abs(zhi)>10000000 ||
      std::abs(p.x/r)>10000000 || std::abs(p.y/r)>10000000) {
    out.reason="QUERY_BUDGET_EXCEEDED"; return out;
  }
  auto fresh=[&](const ObservedVoxel & v) {
    return std::isfinite(v.direct_stamp) && v.direct_stamp>0 &&
      now>=v.direct_stamp && now-v.direct_stamp<=c.max_observation_age;
  };
  const double co=std::cos(p.yaw),si=std::sin(p.yaw);
  std::vector<double> anchor;
  for (int a=0;a<static_cast<int>(n_along);++a) {
    const double along=start+(hl-start)*a/(n_along-1);
    SupportSurfaceResult::Row row;
    row.x=p.x+co*along; row.y=p.y+si*along;
    std::vector<double> row_heights;
    for(int b=0;b<static_cast<int>(n_across);++b) {
      const double across=-hw+2*hw*b/(n_across-1);
      const int x=static_cast<int>(std::floor((p.x+co*along-si*across)/r));
      const int y=static_cast<int>(std::floor((p.y+si*along+co*across)/r));
      ++out.columns;
      std::vector<int> levels;
      for(int z=static_cast<int>(zlo);z<=static_cast<int>(zhi);++z) {
        auto v=query(x,y,z);
        if(v.state!=ObservedState::OCCUPIED || !fresh(v))continue;
        bool free=true;
        for(int k=1;k<=free_layers;++k) {
          auto u=query(x,y,z+k);
          if(u.state!=ObservedState::FREE || !fresh(u)){free=false;break;}
        }
        if(free)levels.push_back(z);
      }
      if(levels.empty()){
        ++out.missing; ++row.missing;
        out.unobserved_columns.push_back({(x+.5)*r,(y+.5)*r,p.ground_z});
        continue;
      }
      for(int z:levels)out.candidates.push_back({(x+.5)*r,(y+.5)*r,(z+.5)*r});
      if(levels.size()!=1){++out.ambiguous;++row.ambiguous;continue;}
      const double z=(levels.front()+.5)*r;
      ++row.observed; row_heights.push_back(z);
      if(a==0)anchor.push_back(z);
      else if(!anchor.empty()) {
        auto sorted=anchor;std::sort(sorted.begin(),sorted.end());
        if(std::abs(z-sorted[sorted.size()/2])>r+1e-6)++out.discontinuous;
      }
    }
    if(!row_heights.empty()) {
      std::sort(row_heights.begin(),row_heights.end());
      row.z=row_heights[row_heights.size()/2];
      out.max_row_spread=std::max(out.max_row_spread,row_heights.back()-row_heights.front());
      if(!out.rows.empty() && out.rows.back().observed) {
        out.max_observed_row_delta=std::max(
          out.max_observed_row_delta,std::abs(row.z-out.rows.back().z));
      }
    }
    out.rows.push_back(row);
  }
  if(anchor.empty()){out.reason="NO_OBSERVED_APPROACH_SUPPORT";return out;}
  std::sort(anchor.begin(),anchor.end());out.ground_z=anchor[anchor.size()/2];
  if(anchor.back()-anchor.front()>r+1e-6)++out.discontinuous;
  if(out.ambiguous)out.reason="AMBIGUOUS_SUPPORT_LEVELS";
  else if(out.discontinuous)out.reason="SUPPORT_STEP_OR_SLOPE";
  else if(out.missing)out.reason="INCOMPLETE_SUPPORT_OBSERVATION";
  else {out.complete=true;out.reason="LEVEL_SUPPORT_CANDIDATE_ONLY";}
  return out;
}
}  // namespace sentry_terrain_bridge
