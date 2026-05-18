#include "velocity_profiler/velocity_profiler.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace velocity_profiler
{

VelocityProfiler::VelocityProfiler(const ProfilerParams & params)
: params_(params)
{
  validateParams();
}

void VelocityProfiler::setParams(const ProfilerParams & params)
{
  params_ = params;
  validateParams();
}

std::vector<StampedWaypoint> VelocityProfiler::generate(
  std::vector<StampedWaypoint> wps, double ego_speed) const
{
  // Empty input is a programming error: throw immediately.
  if (wps.empty()) {
    throw std::invalid_argument("[VelocityProfiler] generate() called with empty waypoints");
  }

  // Single waypoint: goal-reached or fallback stop from the local planner: zero and return.
  if (wps.size() == 1) {
    wps[0].velocity = 0.0;
    wps[0].acceleration = 0.0;
    wps[0].time_from_start = 0.0;
    return wps;
  }

  //  Step 0: geometry
  computeGeometry(wps);

  //  Step 1: speed caps + collect stop indices from is_stop_point
  std::vector<int> stop_indices;
  const auto caps = computeSpeedCaps(wps, stop_indices);

  // Stop points are marked explicitly by the local planner via is_stop_point.
  // Do NOT force a stop at the last waypoint: the trajectory is a receding-horizon
  // window (30 m lookahead), not the full route to the final goal. Adding a phantom
  // stop here causes the backward pass to suppress all velocities across the entire
  // horizon on every tick, preventing the vehicle from reaching cruise speed.

  // Deduplicate and sort
  std::sort(stop_indices.begin(), stop_indices.end());
  stop_indices.erase(std::unique(stop_indices.begin(), stop_indices.end()), stop_indices.end());

  //  Step 2: forward pass
  forwardPass(wps, caps, ego_speed);

  // Step 3: backward pass
  backwardPass(wps, stop_indices);

  // Step 4: jerk smoothing (best effort; it only lowers speeds)
  jerkSmoothing(wps);

  // Step 5: lowering speeds during smoothing can break the acceleration or
  // braking bound, so re-apply both. Each sweep only lowers speeds, so the
  // caps and stop points still hold, and after both sweeps a_max and a_dec
  // hold between every pair of points.
  limitAcceleration(wps);
  limitDeceleration(wps);

  // Step 6: timing
  assignTimings(wps);

  return wps;
}

void VelocityProfiler::computeGeometry(std::vector<StampedWaypoint> & wps) const
{
  const std::size_t N = wps.size();

  wps[0].ds = 0.0;
  wps[0].curvature = 0.0;

  for (std::size_t i = 1; i < N; ++i) {
    const double dx = wps[i].x - wps[i - 1].x;
    const double dy = wps[i].y - wps[i - 1].y;
    const double ds = std::hypot(dx, dy);

    wps[i].ds = std::max(ds, params_.min_ds);

    double dYaw = wps[i].yaw - wps[i - 1].yaw;
    while (dYaw > M_PI) {
      dYaw -= 2.0 * M_PI;
    }
    while (dYaw < -M_PI) {
      dYaw += 2.0 * M_PI;
    }

    wps[i].curvature = (ds > params_.min_ds) ? (dYaw / ds) : 0.0;
  }

  // 3-point moving average to smooth discrete yaw noise
  for (std::size_t i = 1; i + 1 < N; ++i) {
    wps[i].curvature = (wps[i - 1].curvature + wps[i].curvature + wps[i + 1].curvature) / 3.0;
  }
}

std::vector<double> VelocityProfiler::computeSpeedCaps(
  const std::vector<StampedWaypoint> & wps, std::vector<int> & stop_indices) const
{
  const std::size_t N = wps.size();
  std::vector<double> caps(N);

  for (std::size_t i = 0; i < N; ++i) {
    // 1. Hard max speed
    double cap = params_.max_speed;

    // 2. Per-point constraint from the local planner
    //    max_velocity_mps encodes: min(HD-map speed limit, behavior speed limit)
    cap = std::min(cap, wps[i].max_velocity_mps);

    // 3. Curvature-based lateral acceleration limit
    //    v_curve = sqrt(a_lat_max / |kappa|)
    const double abs_kappa = std::abs(wps[i].curvature);
    if (abs_kappa > params_.min_curvature_threshold) {
      const double v_curve = std::sqrt(params_.a_lat_max / abs_kappa);
      cap = std::min(cap, v_curve);
    }

    // 4. Explicit stop point from the local planner
    //    is_stop_point = true when the local planner determined the ego must stop here
    //    (traffic light, goal, etc.)
    if (wps[i].is_stop_point) {
      cap = 0.0;
      stop_indices.push_back(static_cast<int>(i));
    }

    // 5. Cruise speed floor: never cap above cruise if not already capped below it
    cap = std::min(cap, params_.cruise_speed);

    // 6. Hard floor
    caps[i] = std::max(cap, 0.0);
  }

  return caps;
}

void VelocityProfiler::forwardPass(
  std::vector<StampedWaypoint> & wps, const std::vector<double> & caps, double v0) const
{
  const std::size_t N = wps.size();

  wps[0].velocity = std::min(std::max(v0, 0.0), caps[0]);

  for (std::size_t i = 1; i < N; ++i) {
    const double ds = wps[i].ds;
    const double v_prev = wps[i - 1].velocity;
    const double v_acc = std::sqrt(v_prev * v_prev + 2.0 * params_.a_max * ds);
    wps[i].velocity = std::min(v_acc, caps[i]);
  }
}

void VelocityProfiler::backwardPass(
  std::vector<StampedWaypoint> & wps, const std::vector<int> & stop_indices) const
{
  const std::size_t N = wps.size();

  for (const int stop_idx : stop_indices) {
    if (stop_idx < 0 || stop_idx >= static_cast<int>(N)) {
      continue;
    }

    // Force zero at the stop point and everywhere past it (ego must not creep forward).
    for (std::size_t j = static_cast<std::size_t>(stop_idx); j < N; ++j) {
      wps[j].velocity = 0.0;
    }
  }

  // One sweep over the whole profile, not just from stop points: any drop in the
  // speed cap (curve entry, speed-limited zone, creep zone) needs a braking ramp.
  // Example: a forward pass of [8.33, 8.33, 0.50, 0.50] with no stop point would
  // otherwise ask for about 7 g of braking into the 0.50 m/s zone.
  limitDeceleration(wps);
}

void VelocityProfiler::limitAcceleration(std::vector<StampedWaypoint> & wps) const
{
  for (std::size_t i = 1; i < wps.size(); ++i) {
    const double v_prev = wps[i - 1].velocity;
    const double v_acc = std::sqrt(v_prev * v_prev + 2.0 * params_.a_max * wps[i].ds);
    wps[i].velocity = std::min(wps[i].velocity, v_acc);
  }
}

void VelocityProfiler::limitDeceleration(std::vector<StampedWaypoint> & wps) const
{
  // No early exit: a constraint satisfied at point i does not imply it holds at
  // earlier points, which may carry higher speeds from the forward pass.
  for (std::size_t i = wps.size() - 1; i-- > 0; ) {
    const double v_next = wps[i + 1].velocity;
    const double v_dec = std::sqrt(v_next * v_next + 2.0 * params_.a_dec * wps[i + 1].ds);
    wps[i].velocity = std::min(wps[i].velocity, v_dec);
  }
}

void VelocityProfiler::jerkSmoothing(std::vector<StampedWaypoint> & wps) const
{
  const std::size_t N = wps.size();
  if (N < 3) {
    return;
  }

  for (int pass = 0; pass < 2; ++pass) {
    // Forward sweep
    for (std::size_t i = 1; i + 1 < N; ++i) {
      const double ds0 = wps[i].ds;
      const double ds1 = wps[i + 1].ds;
      if (ds0 < params_.min_ds || ds1 < params_.min_ds) {
        continue;
      }

      const double v0 = wps[i - 1].velocity;
      const double v1 = wps[i].velocity;
      const double v2 = wps[i + 1].velocity;

      const double dt0 = ds0 / std::max(0.5 * (v0 + v1), 0.1);
      const double dt1 = ds1 / std::max(0.5 * (v1 + v2), 0.1);
      const double a0 = (v1 - v0) / dt0;
      const double a1 = (v2 - v1) / dt1;
      const double dt = 0.5 * (dt0 + dt1);
      const double jerk = (a1 - a0) / dt;

      if (std::abs(jerk) > params_.j_max) {
        const double da_max = params_.j_max * dt;
        const double a1_clip = std::clamp(a1, a0 - da_max, a0 + da_max);
        const double v2_clip = v1 + a1_clip * dt1;
        wps[i + 1].velocity = std::min(wps[i + 1].velocity, std::max(v2_clip, 0.0));
      }
    }

    // Backward sweep
    for (std::size_t i = N - 2; i >= 1; --i) {
      const double ds0 = wps[i].ds;
      const double ds1 = wps[i + 1].ds;
      if (ds0 < params_.min_ds || ds1 < params_.min_ds) {
        continue;
      }

      const double v0 = wps[i - 1].velocity;
      const double v1 = wps[i].velocity;
      const double v2 = wps[i + 1].velocity;

      const double dt0 = ds0 / std::max(0.5 * (v0 + v1), 0.1);
      const double dt1 = ds1 / std::max(0.5 * (v1 + v2), 0.1);
      const double a0 = (v1 - v0) / dt0;
      const double a1 = (v2 - v1) / dt1;
      const double dt = 0.5 * (dt0 + dt1);
      const double jerk = (a1 - a0) / dt;

      if (std::abs(jerk) > params_.j_max) {
        const double da_max = params_.j_max * dt;
        const double a0_clip = std::clamp(a0, a1 - da_max, a1 + da_max);
        const double v1_clip = v0 + a0_clip * dt0;
        wps[i].velocity = std::min(wps[i].velocity, std::max(v1_clip, 0.0));
      }
    }
  }

  for (auto & wp : wps) {
    wp.velocity = std::max(wp.velocity, 0.0);
  }
}

void VelocityProfiler::assignTimings(std::vector<StampedWaypoint> & wps) const
{
  const std::size_t N = wps.size();
  wps[0].acceleration = 0.0;
  wps[0].time_from_start = 0.0;

  double t = 0.0;
  for (std::size_t i = 1; i < N; ++i) {
    const double v0 = wps[i - 1].velocity;
    const double v1 = wps[i].velocity;
    const double ds = wps[i].ds;

    wps[i].acceleration = (ds > params_.min_ds) ? ((v1 * v1 - v0 * v0) / (2.0 * ds)) : 0.0;

    const double v_avg = 0.5 * (v0 + v1);

    t += (v_avg > 0.01) ? (ds / v_avg) : 0.0;
    wps[i].time_from_start = t;
  }
}

void VelocityProfiler::validateParams() const
{
  std::ostringstream err;
  if (params_.cruise_speed <= 0.0) {
    err << "cruise_speed must be > 0\n";
  }
  if (params_.max_speed <= 0.0) {
    err << "max_speed must be > 0\n";
  }
  if (params_.a_max <= 0.0) {
    err << "a_max must be > 0\n";
  }
  if (params_.a_dec <= 0.0) {
    err << "a_dec must be > 0\n";
  }
  if (params_.j_max <= 0.0) {
    err << "j_max must be > 0\n";
  }
  if (params_.a_lat_max <= 0.0) {
    err << "a_lat_max must be > 0\n";
  }
  if (params_.cruise_speed > params_.max_speed) {
    err << "cruise_speed must be <= max_speed\n";
  }
  if (params_.min_ds <= 0.0) {
    err << "min_ds must be > 0\n";
  }
  if (params_.min_curvature_threshold < 0.0) {
    err << "min_curvature_threshold must be >= 0\n";
  }

  const std::string msg = err.str();
  if (!msg.empty()) {
    throw std::invalid_argument("[VelocityProfiler] Invalid params:\n" + msg);
  }
}

}  // namespace velocity_profiler
