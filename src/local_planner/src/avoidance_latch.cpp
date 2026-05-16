#include "local_planner/avoidance_latch.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace local_planner
{

AvoidanceLatch::AvoidanceLatch(double release_distance)
: release_distance_(release_distance) {}

void AvoidanceLatch::reset()
{
  latched_ = false;
  consecutive_failures_ = 0;
  latched_path_.clear();
  latched_obstacle_.reset();
}

void AvoidanceLatch::latch(
  const std::vector<TrajectoryPoint> & path, const std::vector<Obstacle> & obstacles,
  const Pose & ego)
{
  latched_ = true;
  latched_path_ = path;

  // Store the FARTHEST obstacle circle ahead as the release reference.
  //
  // The 3-circle model places the nearest circle at the rear of the obstacle,
  // which goes "behind" the ego too soon while the ego is still beside it.
  // Using the farthest forward circle ensures the latch holds until the ego
  // has fully passed the front of the obstacle.
  if (!obstacles.empty()) {
    const double cos_h = std::cos(ego.heading);
    const double sin_h = std::sin(ego.heading);

    latched_obstacle_ = *std::max_element(
      obstacles.begin(), obstacles.end(),
      [&ego, cos_h, sin_h](const Obstacle & a, const Obstacle & b) {
        const double fwd_a = (a.x - ego.x) * cos_h + (a.y - ego.y) * sin_h;
        const double fwd_b = (b.x - ego.x) * cos_h + (b.y - ego.y) * sin_h;
        return fwd_a < fwd_b;  // max_element picks the farthest forward
      });
  }
}

bool AvoidanceLatch::recordFailure(const rclcpp::Logger & logger)
{
  ++consecutive_failures_;

  if (consecutive_failures_ >= kMaxFailures) {
    RCLCPP_WARN(
      logger,
      "[AvoidanceLatch] Avoidance permanently failing (%d failures): "
      "releasing latch for WAIT_BEHIND_OBSTACLE",
      kMaxFailures);
    reset();
    return true;
  }

  return false;
}

bool AvoidanceLatch::isObstacleCleared(const Pose & ego) const
{
  if (!latched_obstacle_.has_value()) {
    return true;
  }

  // Use the FIXED obstacle position stored at latch time: NOT the live list.
  //
  // The live list changes as the ego moves (circles rotate in/out of the lookahead
  // window as the ego passes them), which caused the latch to release mid-maneuver
  // when the ego was still beside the obstacle.
  const double cos_h = std::cos(ego.heading);
  const double sin_h = std::sin(ego.heading);

  const double dx = latched_obstacle_->x - ego.x;
  const double dy = latched_obstacle_->y - ego.y;
  const double forward = dx * cos_h + dy * sin_h;

  // Hold latch until the farthest obstacle circle is release_distance_ behind.
  return forward < -release_distance_;
}

std::vector<TrajectoryPoint> AvoidanceLatch::trimFromEgo(const Pose & ego) const
{
  const auto & path = latched_path_;

  if (path.empty()) {
    return {};
  }

  const double cos_h = std::cos(ego.heading);
  const double sin_h = std::sin(ego.heading);

  // First pass: find the closest point by Euclidean distance as a seed.
  std::size_t closest_idx = 0;
  double best_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double d = std::hypot(path[i].pose.x - ego.x, path[i].pose.y - ego.y);
    if (d < best_dist) {
      best_dist = d;
      closest_idx = i;
    }
  }

  // Second pass: walk forward from closest_idx to find first point ahead of ego.
  // Guards against the closest point being slightly behind due to lateral offset.
  for (std::size_t i = closest_idx; i < path.size(); ++i) {
    const double dx = path[i].pose.x - ego.x;
    const double dy = path[i].pose.y - ego.y;
    if (dx * cos_h + dy * sin_h >= 0.0) {
      return std::vector<TrajectoryPoint>(path.begin() + i, path.end());
    }
  }

  // All remaining points are behind the ego (ego has passed the end of the latched path).
  // Extend forward from the last path point so the controller always has a valid
  // ahead target and does not brake unnecessarily.
  const auto & last = path.back();
  const double ext_cos = std::cos(last.pose.heading);
  const double ext_sin = std::sin(last.pose.heading);

  std::vector<TrajectoryPoint> extended;
  extended.reserve(10);
  for (int k = 1; k <= 10; ++k) {
    TrajectoryPoint pt = last;
    pt.pose.x = last.pose.x + k * 1.0 * ext_cos;
    pt.pose.y = last.pose.y + k * 1.0 * ext_sin;
    pt.is_stop_point = false;
    extended.push_back(pt);
  }

  return extended;
}

}  // namespace local_planner
