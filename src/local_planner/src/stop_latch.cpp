#include "local_planner/stop_latch.hpp"

#include <cmath>

namespace local_planner
{

StopLatch::StopLatch(double commit_distance, double creep_speed)
: commit_distance_(commit_distance), creep_speed_(creep_speed)
{
}

void StopLatch::commit(double stop_x, double stop_y, std::string reason)
{
  committed_ = true;
  stop_x_ = stop_x;
  stop_y_ = stop_y;
  stop_reason_ = std::move(reason);
}

bool StopLatch::update(
  const Pose & ego, bool currently_stopping, double ego_speed,
  std::vector<TrajectoryPoint> & trajectory_out, rclcpp::Clock & clock,
  const rclcpp::Logger & logger)
{
  const double dx = stop_x_ - ego.x;
  const double dy = stop_y_ - ego.y;
  const double dist_to_stop = std::hypot(dx, dy);

  // Positive means the committed stop point is still in front of the vehicle.
  const double stop_point_ahead = std::cos(ego.heading) * dx + std::sin(ego.heading) * dy;
  const bool passed_committed_stop = stop_point_ahead < -1.0;

  if (currently_stopping) {
    non_stopping_ticks_ = 0;
  } else {
    ++non_stopping_ticks_;
  }

  // Release condition A: the vehicle has driven past the stop point.
  if (!currently_stopping && passed_committed_stop) {
    committed_ = false;
    non_stopping_ticks_ = 0;
    RCLCPP_INFO(logger, "[StopLatch] Released: committed stop point passed.");
    return false;
  }

  // Release condition B: behavior cleared stably AND vehicle stopped inside commit zone.
  //
  // Never release on a one-cycle behavior dropout.  Release only after BOTH:
  //   1. The behavior has been non-stopping for kRequiredClearTicks ticks, AND
  //   2. The vehicle has come to a full stop (≤ 0.20 m/s) within the commit zone.
  // Condition 2 prevents the latch from firing when behavior briefly drops out
  // while the vehicle is still moving.
  const bool behavior_cleared_stably = non_stopping_ticks_ >= kRequiredClearTicks;
  const bool stopped_near = dist_to_stop <= commit_distance_ && ego_speed <= 0.20;

  if (!currently_stopping && behavior_cleared_stably && stopped_near) {
    committed_ = false;
    non_stopping_ticks_ = 0;
    RCLCPP_INFO(logger, "[StopLatch] Released: behavior cleared after full stop.");
    return false;
  }

  // Still latched: build a creep-to-stop trajectory toward the committed point.
  Pose committed_pose = ego;
  committed_pose.x = stop_x_;
  committed_pose.y = stop_y_;

  trajectory_out.clear();
  trajectory_out.reserve(5);

  constexpr std::size_t kStopHoldPoints = 5;
  for (std::size_t i = 0; i < kStopHoldPoints; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(kStopHoldPoints - 1);

    TrajectoryPoint pt;
    pt.pose.x = ego.x + ratio * dx;
    pt.pose.y = ego.y + ratio * dy;
    pt.pose.z = ego.z;
    pt.pose.heading = ego.heading;
    pt.max_velocity_mps = static_cast<float>(creep_speed_);
    pt.is_stop_point = false;
    pt.stop_reason = {};

    if (i + 1 == kStopHoldPoints) {
      // Final point: full stop at the committed location.
      pt.pose.x = committed_pose.x;
      pt.pose.y = committed_pose.y;
      pt.max_velocity_mps = 0.0;
      pt.is_stop_point = true;
      pt.stop_reason = stop_reason_;
    }

    trajectory_out.push_back(pt);
  }

  RCLCPP_INFO_THROTTLE(
    logger, clock, 1000,
    "[StopLatch] Holding committed stop. behavior_active=%d clear_ticks=%d dist=%.2f ahead=%.2f",
    currently_stopping, non_stopping_ticks_, dist_to_stop, stop_point_ahead);

  return true;
}

void StopLatch::maybeCommit(
  const Pose & ego, const std::vector<TrajectoryPoint> & trajectory, const rclcpp::Logger & logger,
  std::string reason)
{
  if (trajectory.empty()) {
    return;
  }

  const auto & stop_pt = trajectory.back();
  const double dist_to_stop = std::hypot(stop_pt.pose.x - ego.x, stop_pt.pose.y - ego.y);

  if (dist_to_stop > commit_distance_) {
    return;
  }

  commit(stop_pt.pose.x, stop_pt.pose.y, std::move(reason));

  RCLCPP_INFO(
    logger, "[StopLatch] Committed at (%.1f, %.1f), dist=%.2f m", stop_x_, stop_y_, dist_to_stop);
}

}  // namespace local_planner
