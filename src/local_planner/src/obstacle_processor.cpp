#include "local_planner/obstacle_processor.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace local_planner
{

ObstacleProcessor::ObstacleProcessor(ObstacleProcessorParams params, VehicleParams vehicle)
: params_(params), vehicle_(vehicle)
{
}

void ObstacleProcessor::update(
  const derived_object_msgs::msg::ObjectArray & msg, double ego_x, double ego_y, double ego_yaw)
{
  const double cos_h = std::cos(ego_yaw);
  const double sin_h = std::sin(ego_yaw);

  const double ego_filter_rad = vehicle_.length * params_.ego_filter_multiplier;

  // Maximum lateral gate: whichever corridor is wider (avoidance or lane).
  const double lateral_gate = std::max(
    vehicle_.width * params_.corridor_width_multiplier * params_.avoidance_corridor_multiplier,
    params_.min_avoidance_corridor_half_width);

  obstacles_.clear();

  for (const auto & obj : msg.objects) {
    const double ox = obj.pose.position.x;
    const double oy = obj.pose.position.y;

    const double dx = ox - ego_x;
    const double dy = oy - ego_y;

    // Ego self-filter
    if (std::hypot(dx, dy) < ego_filter_rad) {
      continue;
    }

    // Ego-frame projection
    const double longitudinal = dx * cos_h + dy * sin_h;
    const double lateral = -dx * sin_h + dy * cos_h;

    // Discard objects behind the rear axle.
    if (longitudinal < -vehicle_.length) {
      continue;
    }
    // Discard objects beyond the forward lookahead.
    if (longitudinal > params_.lookahead) {
      continue;
    }
    // Discard objects outside the widest corridor (fast lateral cull).
    if (std::abs(lateral) > lateral_gate + params_.inflation) {
      continue;
    }

    double radius = params_.inflation;

    if (obj.shape.dimensions.size() >= 2) {
      const double d0 = obj.shape.dimensions[0];
      const double d1 = obj.shape.dimensions[1];
      const double obs_length = std::max(d0, d1);
      const double obs_width = std::min(d0, d1);

      radius = 0.5 * obs_width + params_.inflation;

      // Place 3 circles along the EGO heading, not the obstacle's own yaw.
      //
      // Reason: the obstacle yaw from /carla/objects is often identity (0°)
      // regardless of how the obstacle is oriented on the road. Using the
      // obstacle's yaw blindly places circles east-west when the road runs
      // north-south, so the circles miss the route entirely.
      // Using the ego heading aligns the circles along the direction of travel,
      // which is always correct for a static parked car roughly aligned with the road.
      const std::array<double, 3> offsets{-0.35 * obs_length, 0.0, 0.35 * obs_length};

      for (const double s : offsets) {
        obstacles_.push_back({ox + s * cos_h, oy + s * sin_h, radius});
      }
      continue;
    }

    obstacles_.push_back({ox, oy, radius});
  }
}

bool ObstacleProcessor::obstacleAhead(const Pose & ego_pose, bool avoidance_active) const
{
  const double cos_h = std::cos(ego_pose.heading);
  const double sin_h = std::sin(ego_pose.heading);

  // Lateral half-width matches the route corridor used by buildRouteCorridor().
  const double c_half = avoidance_active ? std::max(
                                             vehicle_.width * params_.corridor_width_multiplier *
                                               params_.avoidance_corridor_multiplier,
                                             params_.min_avoidance_corridor_half_width)
                                         : vehicle_.width * params_.corridor_width_multiplier;

  for (const auto & obs : obstacles_) {
    const double dx = obs.x - ego_pose.x;
    const double dy = obs.y - ego_pose.y;

    const double longitudinal = dx * cos_h + dy * sin_h;
    if (longitudinal < 0.0 || longitudinal > params_.lookahead) {
      continue;
    }

    const double lateral = -dx * sin_h + dy * cos_h;
    if (std::abs(lateral) > c_half + obs.radius) {
      continue;
    }

    return true;
  }

  return false;
}

}  // namespace local_planner
