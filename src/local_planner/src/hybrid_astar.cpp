#include "local_planner/hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>

namespace local_planner
{

HybridAStar::HybridAStar(const VehicleParams & vehicle_params, const HybridAStarParams & params)
: vehicle_(vehicle_params),
  params_(params),
  heading_bin_size_(kTwoPi / static_cast<double>(params.heading_bins))
{
}

HybridAStarResult HybridAStar::plan(
  const Pose & start, const Pose & goal, const std::vector<Obstacle> & obstacles,
  const RouteCorridor * corridor, double max_velocity_mps) const
{
  if (!isInsideCorridor(start, corridor)) {
    return {false, {}, "Start pose is outside route corridor"};
  }
  if (!isInsideCorridor(goal, corridor)) {
    return {false, {}, "Goal pose is outside route corridor"};
  }
  if (!isCollisionFree(start, obstacles)) {
    return {false, {}, "Start position is inside an obstacle"};
  }
  if (!isCollisionFree(goal, obstacles)) {
    return {false, {}, "Goal position is inside an obstacle"};
  }
  if (std::hypot(goal.x - start.x, goal.y - start.y) < params_.grid_resolution) {
    return {false, {}, "Start and goal are within same grid cell"};
  }

  std::priority_queue<HybridNode, std::vector<HybridNode>, std::greater<HybridNode>> open_list;

  std::vector<HybridNode> closed_list;
  closed_list.reserve(static_cast<std::size_t>(params_.max_iterations));

  std::unordered_map<std::size_t, int> visited;
  visited.reserve(static_cast<std::size_t>(params_.max_iterations));

  HybridNode start_node;
  start_node.pose = {start.x, start.y, start.z, normalizeAngle(start.heading)};
  start_node.g = 0.0;
  start_node.f = heuristic(start_node.pose, goal);
  start_node.grid_x = toGridX(start.x);
  start_node.grid_y = toGridY(start.y);
  start_node.grid_h = toGridH(start.heading);
  start_node.parent_index = -1;

  open_list.push(start_node);

  const std::vector<double> steering_angles = generateSteeringAngles();
  int iterations = 0;

  while (!open_list.empty() && iterations < params_.max_iterations) {
    ++iterations;

    HybridNode current = open_list.top();
    open_list.pop();

    const std::size_t key = toGridKey(current.pose);

    if (visited.count(key)) {
      continue;
    }

    const int current_idx = static_cast<int>(closed_list.size());
    closed_list.push_back(current);
    visited[key] = current_idx;

    if (isGoal(current, goal)) {
      return {true, extractPath(closed_list, current_idx, max_velocity_mps), ""};
    }

    for (const double steer : steering_angles) {
      HybridNode child = applyMotion(current, steer);
      child.parent_index = current_idx;
      child.pose.z = current.pose.z;

      if (visited.count(toGridKey(child.pose))) {
        continue;
      }

      if (!isMotionInsideCorridor(current.pose, steer, corridor)) {
        continue;
      }

      if (!isMotionCollisionFree(current.pose, steer, obstacles)) {
        continue;
      }

      child.g = current.g + params_.step_size;
      child.f = child.g + heuristic(child.pose, goal);
      open_list.push(child);
    }
  }

  return {
    false,
    {},
    "Max iterations (" + std::to_string(params_.max_iterations) + ") reached: no path found"};
}

bool HybridAStar::isCollisionFree(const Pose & pose, const std::vector<Obstacle> & obstacles) const
{
  const double cos_h = std::cos(pose.heading);
  const double sin_h = std::sin(pose.heading);

  const double half_len = vehicle_.length * 0.5;
  const std::array<double, 5> longitudinal_samples{
    {-half_len, -half_len * 0.5, 0.0, half_len * 0.5, half_len}};

  const double vehicle_radius = vehicle_.width * 0.5;

  for (const double s : longitudinal_samples) {
    const double px = pose.x + s * cos_h;
    const double py = pose.y + s * sin_h;

    for (const auto & obs : obstacles) {
      const double min_dist = obs.radius + vehicle_radius;
      if (std::hypot(px - obs.x, py - obs.y) < min_dist) {
        return false;
      }
    }
  }

  return true;
}

Pose HybridAStar::integrate(const Pose & start, double steering_rad, double distance) const
{
  const double steer =
    std::clamp(steering_rad, -vehicle_.max_steering_angle, vehicle_.max_steering_angle);

  Pose out;
  out.z = start.z;

  const double tan_delta = std::tan(steer);

  if (std::abs(tan_delta) < 1e-6) {
    // Straight-line motion: avoid division by near-zero radius
    out.x = start.x + distance * std::cos(start.heading);
    out.y = start.y + distance * std::sin(start.heading);
    out.heading = start.heading;
    return out;
  }

  // Proper rear-axle bicycle model arc integration.
  // radius = wheelbase / tan(steer)
  // The rear axle traces a circular arc of this radius.
  // x_new = x + R * (sin(θ + dθ) − sin(θ))
  // y_new = y − R * (cos(θ + dθ) − cos(θ))
  // θ_new = θ + dθ,  where dθ = distance / R
  const double radius = vehicle_.wheelbase / tan_delta;
  const double dtheta = distance / radius;

  out.x = start.x + radius * (std::sin(start.heading + dtheta) - std::sin(start.heading));
  out.y = start.y - radius * (std::cos(start.heading + dtheta) - std::cos(start.heading));
  out.heading = normalizeAngle(start.heading + dtheta);

  return out;
}

bool HybridAStar::isMotionCollisionFree(
  const Pose & start, double steering_rad, const std::vector<Obstacle> & obstacles) const
{
  constexpr int kSamples = 5;

  for (int i = 1; i <= kSamples; ++i) {
    const double distance =
      params_.step_size * static_cast<double>(i) / static_cast<double>(kSamples);

    const Pose sample_pose = integrate(start, steering_rad, distance);
    if (!isCollisionFree(sample_pose, obstacles)) {
      return false;
    }
  }

  return true;
}

bool HybridAStar::isMotionInsideCorridor(
  const Pose & start, double steering_rad, const RouteCorridor * corridor) const
{
  // No corridor constraint: always inside
  if (corridor == nullptr || corridor->centerline.empty()) {
    return true;
  }

  // Sample 5 points along the arc (identical pattern to isMotionCollisionFree)
  // to catch any boundary crossing mid-step.
  constexpr int kSamples = 5;
  for (int i = 1; i <= kSamples; ++i) {
    const double distance =
      params_.step_size * static_cast<double>(i) / static_cast<double>(kSamples);

    const Pose sample_pose = integrate(start, steering_rad, distance);

    if (!isInsideCorridor(sample_pose, corridor)) {
      return false;
    }
  }

  return true;
}

bool HybridAStar::isInsideCorridor(const Pose & pose, const RouteCorridor * corridor) const
{
  if (corridor == nullptr || corridor->centerline.empty()) {
    return true;
  }

  // Precompute longitudinal direction
  const double cos_h = std::cos(pose.heading);
  const double sin_h = std::sin(pose.heading);
  const double half_w = vehicle_.width * 0.5;

  const double half_len = vehicle_.length * 0.5;
  const std::array<double, 5> long_offsets{
    {-half_len, -half_len * 0.5, 0.0, half_len * 0.5, half_len}};

  // Lateral offsets: center + left edge + right edge
  const std::array<double, 3> lat_offsets{{0.0, half_w, -half_w}};

  // Precompute lateral direction (perpendicular to heading)
  const double lat_cos = -sin_h;  // perpendicular: rotate heading 90°
  const double lat_sin = cos_h;

  for (const double s : long_offsets) {
    const double bx = pose.x + s * cos_h;  // base longitudinal point
    const double by = pose.y + s * sin_h;

    for (const double lat : lat_offsets) {
      const double px = bx + lat * lat_cos;
      const double py = by + lat * lat_sin;

      // Find nearest corridor centerline point to this footprint sample.
      // Use that point's per-point half_width for the boundary check.
      // This correctly handles roads that narrow along the avoidance segment.
      double best_dist = std::numeric_limits<double>::infinity();
      double local_half_width = corridor->half_width;  // global fallback

      for (const auto & pt : corridor->centerline) {
        const double d = std::hypot(px - pt.x, py - pt.y);
        if (d < best_dist) {
          best_dist = d;
          // Use per-point width if set (>0), else fall back to global
          local_half_width = (pt.half_width > 0.01) ? pt.half_width : corridor->half_width;
        }
      }

      // ANY footprint point outside its local road boundary → reject
      if (best_dist > local_half_width) {
        return false;
      }
    }
  }

  return true;
}

HybridNode HybridAStar::applyMotion(const HybridNode & node, double steering_rad) const
{
  const double steer =
    std::clamp(steering_rad, -vehicle_.max_steering_angle, vehicle_.max_steering_angle);

  const Pose next_pose = integrate(node.pose, steer, params_.step_size);

  HybridNode child;
  child.pose = next_pose;
  child.steering_angle = steer;
  child.grid_x = toGridX(child.pose.x);
  child.grid_y = toGridY(child.pose.y);
  child.grid_h = toGridH(child.pose.heading);

  return child;
}

std::vector<double> HybridAStar::generateSteeringAngles() const
{
  std::vector<double> angles;
  angles.reserve(static_cast<std::size_t>(params_.steering_samples));

  const int n = params_.steering_samples;
  const double max_s = vehicle_.max_steering_angle;

  for (int i = 0; i < n; ++i) {
    const double t = (n == 1) ? 0.0 : static_cast<double>(i) / static_cast<double>(n - 1);
    angles.push_back(-max_s + t * 2.0 * max_s);
  }
  return angles;
}

double HybridAStar::heuristic(const Pose & node_pose, const Pose & goal) const
{
  return params_.heuristic_weight * dubinsLength(node_pose, goal, vehicle_.min_turning_radius);
}

double HybridAStar::dubinsLength(const Pose & start, const Pose & goal, double radius) const
{
  const double dx = (goal.x - start.x) / radius;
  const double dy = (goal.y - start.y) / radius;
  const double d = std::hypot(dx, dy);
  const double theta = std::atan2(dy, dx);

  const double alpha = mod2pi(start.heading - theta);
  const double beta = mod2pi(goal.heading - theta);

  double best = std::numeric_limits<double>::infinity();
  best = std::min(best, dubinsLSL(d, alpha, beta));
  best = std::min(best, dubinsRSR(d, alpha, beta));
  best = std::min(best, dubinsLSR(d, alpha, beta));
  best = std::min(best, dubinsRSL(d, alpha, beta));
  best = std::min(best, dubinsRLR(d, alpha, beta));
  best = std::min(best, dubinsLRL(d, alpha, beta));

  if (std::isinf(best)) {
    return std::hypot(goal.x - start.x, goal.y - start.y);
  }
  return best * radius;
}

double HybridAStar::dubinsLSL(double d, double alpha, double beta) const
{
  const double tmp =
    2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (std::sin(alpha) - std::sin(beta));
  if (tmp < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = std::sqrt(tmp);
  const double phi =
    std::atan2(std::cos(beta) - std::cos(alpha), d + std::sin(alpha) - std::sin(beta));
  const double t = mod2pi(-alpha + phi);
  const double q = mod2pi(beta - phi);
  return t + p + q;
}

double HybridAStar::dubinsRSR(double d, double alpha, double beta) const
{
  const double tmp =
    2.0 + d * d - 2.0 * std::cos(alpha - beta) + 2.0 * d * (std::sin(beta) - std::sin(alpha));
  if (tmp < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = std::sqrt(tmp);
  const double phi =
    std::atan2(std::cos(alpha) - std::cos(beta), d - std::sin(alpha) + std::sin(beta));
  const double t = mod2pi(alpha - phi);
  const double q = mod2pi(-beta + phi);
  return t + p + q;
}

double HybridAStar::dubinsLSR(double d, double alpha, double beta) const
{
  const double tmp =
    -2.0 + d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (std::sin(alpha) + std::sin(beta));
  if (tmp < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = std::sqrt(tmp);
  const double phi =
    std::atan2(-std::cos(alpha) - std::cos(beta), d + std::sin(alpha) + std::sin(beta)) -
    std::atan2(-2.0, p);
  const double t = mod2pi(-alpha + phi);
  const double q = mod2pi(-beta + phi);
  return t + p + q;
}

double HybridAStar::dubinsRSL(double d, double alpha, double beta) const
{
  const double tmp =
    d * d - 2.0 + 2.0 * std::cos(alpha - beta) - 2.0 * d * (std::sin(alpha) + std::sin(beta));
  if (tmp < 0.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = std::sqrt(tmp);
  const double phi =
    std::atan2(std::cos(alpha) + std::cos(beta), d - std::sin(alpha) - std::sin(beta)) -
    std::atan2(2.0, p);
  const double t = mod2pi(alpha - phi);
  const double q = mod2pi(beta - phi);
  return t + p + q;
}

double HybridAStar::dubinsRLR(double d, double alpha, double beta) const
{
  const double tmp =
    (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (std::sin(alpha) - std::sin(beta))) /
    8.0;

  if (std::abs(tmp) > 1.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = mod2pi(kTwoPi - std::acos(tmp));
  const double phi =
    std::atan2(std::cos(alpha) - std::cos(beta), d - std::sin(alpha) + std::sin(beta));
  const double t = mod2pi(alpha - phi + mod2pi(p / 2.0));
  const double q = mod2pi(alpha - beta - t + mod2pi(p));
  return t + p + q;
}

double HybridAStar::dubinsLRL(double d, double alpha, double beta) const
{
  const double tmp =
    (6.0 - d * d + 2.0 * std::cos(alpha - beta) + 2.0 * d * (-std::sin(alpha) + std::sin(beta))) /
    8.0;

  if (std::abs(tmp) > 1.0) {
    return std::numeric_limits<double>::infinity();
  }

  const double p = mod2pi(kTwoPi - std::acos(tmp));
  const double phi =
    std::atan2(-std::cos(alpha) + std::cos(beta), d + std::sin(alpha) - std::sin(beta));
  const double t = mod2pi(-alpha + phi + mod2pi(p / 2.0));
  const double q = mod2pi(beta - alpha + t + mod2pi(p));
  return t + p + q;
}

std::size_t HybridAStar::toGridKey(const Pose & pose) const
{
  constexpr int kBias = 32768;
  const auto gx = static_cast<std::size_t>((toGridX(pose.x) + kBias) & 0xFFFF);
  const auto gy = static_cast<std::size_t>((toGridY(pose.y) + kBias) & 0xFFFF);
  const auto gh = static_cast<std::size_t>(toGridH(pose.heading) & 0xFFFF);
  return (gx << 32) | (gy << 16) | gh;
}

int HybridAStar::toGridX(double x) const
{
  return static_cast<int>(std::floor(x / params_.grid_resolution));
}

int HybridAStar::toGridY(double y) const
{
  return static_cast<int>(std::floor(y / params_.grid_resolution));
}

int HybridAStar::toGridH(double heading) const
{
  return static_cast<int>(std::floor(normalizeAngle(heading) / heading_bin_size_)) %
         params_.heading_bins;
}

bool HybridAStar::isGoal(const HybridNode & node, const Pose & goal) const
{
  const double pos_err = std::hypot(node.pose.x - goal.x, node.pose.y - goal.y);
  const double hdg_err = std::abs(normalizeAngleSigned(node.pose.heading - goal.heading));
  return (pos_err < params_.goal_position_tolerance) && (hdg_err < params_.goal_heading_tolerance);
}

std::vector<TrajectoryPoint> HybridAStar::extractPath(
  const std::vector<HybridNode> & closed_list, int goal_idx, double max_velocity_mps) const
{
  std::vector<const HybridNode *> chain;
  int idx = goal_idx;
  while (idx >= 0) {
    chain.push_back(&closed_list[static_cast<std::size_t>(idx)]);
    idx = closed_list[static_cast<std::size_t>(idx)].parent_index;
  }
  std::reverse(chain.begin(), chain.end());

  std::vector<TrajectoryPoint> path;
  path.reserve(chain.size());

  for (const auto * node : chain) {
    TrajectoryPoint pt;
    pt.pose = node->pose;
    pt.curvature = std::tan(node->steering_angle) / vehicle_.wheelbase;
    pt.max_velocity_mps = max_velocity_mps;  // behavior speed constraint
    pt.is_stop_point = false;                // set by caller if needed
    pt.stop_reason = {};
    path.push_back(pt);
  }
  return path;
}

bool HybridAStar::isTrajectorySafe(
  const std::vector<TrajectoryPoint> & path, const std::vector<Obstacle> & obstacles,
  const RouteCorridor * corridor) const
{
  for (const auto & pt : path) {
    if (!isCollisionFree(pt.pose, obstacles)) {
      return false;
    }
    if (!isInsideCorridor(pt.pose, corridor)) {
      return false;
    }
  }
  return true;
}

double HybridAStar::normalizeAngle(double angle) const
{
  // Wrap to [0, 2π): used for grid indexing and node storage
  return angle - kTwoPi * std::floor(angle / kTwoPi);
}

double HybridAStar::normalizeAngleSigned(double angle) const
{
  // Wrap to [-π, π): used for heading error computation only
  return angle - kTwoPi * std::floor((angle + M_PI) / kTwoPi);
}

double HybridAStar::mod2pi(double angle) const
{
  const double v = std::fmod(angle, kTwoPi);
  return (v < 0.0) ? v + kTwoPi : v;
}

}  // namespace local_planner
