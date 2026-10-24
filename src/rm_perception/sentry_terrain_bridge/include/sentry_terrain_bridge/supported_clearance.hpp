#pragma once
#include <sentry_terrain_bridge/support_surface.hpp>
#include <map>

namespace sentry_terrain_bridge {
// A local, observed ramp model, NOT a robot climbing-capability model.
// No support holes are filled: fitted height only selects where to query;
// validateClearance still requires directly observed occupied support and free body.
struct SupportedClearance {
  ClearanceResult clearance;
  bool fitted{false};
  bool body_evaluated{false};
  std::string model_reason{"NO_OBSERVED_SUPPORT_MODEL"};
  double ground_z{0.0}, slope{0.0}, max_residual{0.0};
  bool segmented{false};
  double break_along{0.0}, slope_after{0.0};
  double approach_max_residual{0.0};
  std::vector<ClearanceResult::Cell> model_outliers;
  size_t queries{0};
  SupportedClearance() { clearance.reason="NOT_EVALUATED"; }
};

inline double supportMedian(std::vector<double> v) {
  std::sort(v.begin(), v.end());
  return v[v.size()/2];
}

inline SupportedClearance validateSupportedClearance(
  const ClearanceConfig & c, const PassageProbe & p, double r, double now,
  const std::function<ObservedVoxel(int,int,int)> & query,
  double max_slope_radians)
{
  SupportedClearance out;
  if (!validClearanceConfig(c) || !query || !std::isfinite(r) || r<=0 ||
      !std::isfinite(max_slope_radians) || max_slope_radians<0 || max_slope_radians>.6) {
    out.model_reason="INVALID_SUPPORT_CONFIGURATION";
    return out;
  }
  // One aggregate budget, including support search and final body checks.
  bool exhausted=false;
  std::map<std::array<int,3>,ObservedVoxel> cache;
  auto sampled=[&](int x,int y,int z) {
    const std::array<int,3> key{x,y,z};
    auto it=cache.find(key);
    if(it!=cache.end())return it->second;
    if(out.queries>=c.max_voxel_checks){exhausted=true;return ObservedVoxel{};}
    ++out.queries;
    return cache.emplace(key,query(x,y,z)).first->second;
  };
  const auto surface=estimateSupportSurface(c,p,r,now,sampled);
  const double co=std::cos(p.yaw), si=std::sin(p.yaw);
  std::vector<std::array<double,2>> rows;
  // Require transverse support in each fitting row. Sparse samples must not
  // turn a wall edge into a ramp; missing cells remain failures in body validation.
  for(const auto & row:surface.rows) {
    const auto total=row.observed+row.missing+row.ambiguous;
    if(total && row.observed*2>=total && row.ambiguous==0)
      rows.push_back({co*(row.x-p.x)+si*(row.y-p.y),row.z});
  }
  if(exhausted){out.model_reason="QUERY_BUDGET_EXCEEDED";return out;}
  if(rows.size()<4 || rows.back()[0]-rows.front()[0]<c.robot_length) return out;
  if(rows.size()>512){out.model_reason="QUERY_BUDGET_EXCEEDED";return out;}
  // Robust longitudinal fit; a small raised object cannot pull the floor up.
  std::vector<double> slopes;
  for(size_t i=0;i<rows.size();++i)for(size_t j=i+1;j<rows.size();++j)
    if(rows[j][0]-rows[i][0]>=3*r)
      slopes.push_back((rows[j][1]-rows[i][1])/(rows[j][0]-rows[i][0]));
  if(slopes.empty())return out;
  out.slope=supportMedian(slopes);
  std::vector<double> intercepts;
  for(const auto & row:rows)intercepts.push_back(row[1]-out.slope*row[0]);
  out.ground_z=supportMedian(intercepts);
  // A continuous hinge represents ramp -> level (or level -> ramp), never a
  // discontinuous stair. Fit only measured rows; missing support is STILL
  // queried independently below. Require substantial improvement to avoid
  // fitting a bend to voxel quantization noise on an otherwise straight ramp.
  double intercept=out.ground_z, hinge_delta=0.0, best_error=0.0;
  for(const auto & row:rows) {
    const double e=row[1]-intercept-out.slope*row[0];best_error+=e*e;
  }
  const double original_error=best_error;
  // BIC penalty: straight model has 2 parameters; a searched hinge has 4.
  // This avoids an arbitrary per-recording improvement percentage.
  const double hinge_error_ratio=std::exp(-2.0*std::log(static_cast<double>(rows.size()))/rows.size());
  for(size_t k=3;k+3<rows.size();++k) {
    const double bend=rows[k][0];
    if(bend-rows.front()[0]<3*r || rows.back()[0]-bend<3*r)continue;
    double m[3][4]{};
    for(const auto & row:rows) {
      const double v[3]={1.0,row[0],std::max(0.0,row[0]-bend)};
      for(int i=0;i<3;++i) {
        for(int j=0;j<3;++j)m[i][j]+=v[i]*v[j];
        m[i][3]+=v[i]*row[1];
      }
    }
    bool singular=false;
    for(int i=0;i<3;++i) {
      int pivot=i;for(int j=i+1;j<3;++j)if(std::abs(m[j][i])>std::abs(m[pivot][i]))pivot=j;
      if(std::abs(m[pivot][i])<1e-10){singular=true;break;}
      for(int j=0;j<4;++j)std::swap(m[i][j],m[pivot][j]);
      const double d=m[i][i];for(int j=i;j<4;++j)m[i][j]/=d;
      for(int a=0;a<3;++a)if(a!=i) {
        const double f=m[a][i];for(int j=i;j<4;++j)m[a][j]-=f*m[i][j];
      }
    }
    if(singular)continue;
    const double a=m[0][3],b=m[1][3],d=m[2][3];
    if(std::atan(std::max(std::abs(b),std::abs(b+d)))>max_slope_radians)continue;
    double error=0;
    for(const auto & row:rows) {
      const double e=row[1]-a-b*row[0]-d*std::max(0.0,row[0]-bend);error+=e*e;
    }
    if(error<best_error && error<original_error*hinge_error_ratio) {
      best_error=error;intercept=a;out.slope=b;hinge_delta=d;
      out.segmented=true;out.break_along=bend;
    }
  }
  out.slope_after=out.slope+hinge_delta;
  auto predictedHeight=[&](double along) {
    return intercept+out.slope*along+hinge_delta*std::max(0.0,along-out.break_along);
  };
  out.ground_z=predictedHeight(0.0);
  const double max_grade=std::max(std::abs(out.slope),std::abs(out.slope_after));
  out.fitted=true;
  if(std::atan(max_grade)>max_slope_radians+1e-9) {
    out.model_reason="SUPPORT_SLOPE_LIMIT";return out;
  }
  // Every candidate IN THE SWEPT BODY must agree, not just fit inliers.
  // The extra rear strip helps estimate the surface but is not part of this
  // collision query. Report its residual separately instead of vetoing a
  // forward segment because of an object behind the tested footprint.
  const double half_sweep=(p.travel_length+c.robot_length)/2+c.horizontal_margin;
  const double cell_padding=r*.5*(std::abs(co)+std::abs(si));
  for(const auto & cell:surface.candidates) {
    const double along=co*(cell.x-p.x)+si*(cell.y-p.y);
    const double predicted=predictedHeight(along);
    if(std::abs(along)>half_sweep+cell_padding) {
      out.approach_max_residual=std::max(out.approach_max_residual,std::abs(cell.z-predicted));
      continue;
    }
    out.max_residual=std::max(out.max_residual,std::abs(cell.z-predicted));
    if(std::abs(cell.z-predicted)>.5*r+max_grade*r+1e-6)out.model_outliers.push_back(cell);
  }
  const double tolerance=.5*r+max_grade*r+1e-6;
  // Multiple candidate decks inside the sweep appear in the residual check;
  // do not use the estimator's aggregate ambiguity from the auxiliary strip.
  const bool nonplanar=out.max_residual>tolerance;
  auto fresh=[&](const ObservedVoxel & v) {
    return std::isfinite(v.direct_stamp) && v.direct_stamp>0 && now>=v.direct_stamp &&
           now-v.direct_stamp<=c.max_observation_age;
  };
  bool inconsistent=false;
  auto height=[&](int x,int y) {
    const double predicted=predictedHeight(co*((x+.5)*r-p.x)+si*((y+.5)*r-p.y));
    const int center=static_cast<int>(std::floor(predicted/r));
    std::vector<int> candidates;
    // The fit never substitutes for a measured support voxel. Accept only a
    // nearby measured surface; higher occupied voxels remain body obstacles.
    for(int z=center-1;z<=center+1;++z) {
      const auto v=sampled(x,y,z);
      const auto above=sampled(x,y,z+1);
      if(v.state==ObservedState::OCCUPIED && fresh(v) &&
         above.state==ObservedState::FREE && fresh(above) &&
         std::abs((z+.5)*r-predicted)<=tolerance)candidates.push_back(z);
    }
    if(candidates.size()>1){inconsistent=true;return predicted;}
    return candidates.empty()?predicted:(candidates.front()+.5)*r;
  };
  auto adjusted=c;
  // Conservative vertical envelope for a rigid body over a local ramp.
  // This is not a suspension/step/transition simulator.
  adjusted.vertical_margin+=max_grade*c.robot_length;
  auto model_probe=p;model_probe.ground_z=out.ground_z;
  out.body_evaluated=true;
  out.clearance=validateClearance(adjusted,model_probe,r,now,sampled,height);
  out.model_reason=out.segmented?"OBSERVED_CONTINUOUS_SUPPORT_SEGMENTS":"OBSERVED_LOCAL_SUPPORT_PLANE";
  // Retain collision counts for diagnosis even when the plane is rejected.
  // Never turn an outlier into free space to make a model fit.
  if(nonplanar || inconsistent || exhausted) {
    out.clearance.geometry_pass=false;
    out.model_reason=exhausted?"QUERY_BUDGET_EXCEEDED":
      (inconsistent?"AMBIGUOUS_SUPPORT_VOXELS":"NONPLANAR_OR_AMBIGUOUS_SUPPORT");
    out.clearance.reason=out.model_reason;
  }
  return out;
}
} // namespace sentry_terrain_bridge
