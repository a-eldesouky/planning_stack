#include "behavior_planner/behavior_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace behavior_planner
{

// Free function

const char * behaviorModeToString(BehaviorMode mode)
{
  switch (mode) {
    case BehaviorMode::FollowLane:
      return "FOLLOW_LANE";
    case BehaviorMode::FollowLeadingVehicle:
      return "FOLLOW_LEADING_VEHICLE";
    case BehaviorMode::AvoidStaticObstacle:
      return "AVOID_STATIC_OBSTACLE";
    case BehaviorMode::StopAtTrafficLight:
      return "STOP_AT_TRAFFIC_LIGHT";
    case BehaviorMode::WaitBehindObstacle:
      return "WAIT_BEHIND_OBSTACLE";
    default:
      return "UNKNOWN";
  }
}

// BehaviorContext: update methods

void BehaviorContext::resetAvoidanceState()
{
  avoidance_latched = false;
  avoidance_permanently_blocked = false;
  avoidance_stall_ticks = 0;
  avoidance_last_fwd = 0.0;
  latched_obstacle_x = 0.0;
  latched_obstacle_y = 0.0;
  latched_obstacle_half_length = 2.25;
  latched_obstacle_forward_dist = 0.0;
}

void BehaviorContext::updateHysteresis()
{
  // Leading vehicle hysteresis.
  if (has_leading_vehicle) {
    leading_vehicle_count = std::min(
      leading_vehicle_count + 1,
      hysteresis.leading_vehicle_confirm_ticks + hysteresis.exit_hysteresis_ticks);
  } else {
    leading_vehicle_count = std::max(leading_vehicle_count - 1, 0);
  }

  stable_leading_vehicle_detected =
    leading_vehicle_count >= hysteresis.leading_vehicle_confirm_ticks;

  // Static obstacle hysteresis.
  const bool is_static = has_leading_vehicle && leading_vehicle.has_value() &&
                         leading_vehicle->speed < params.static_speed_threshold;

  if (is_static) {
    static_obstacle_count = std::min(
      static_obstacle_count + 1,
      hysteresis.static_obstacle_confirm_ticks + hysteresis.exit_hysteresis_ticks);
  } else {
    static_obstacle_count = std::max(static_obstacle_count - 1, 0);
  }

  stable_static_obstacle = static_obstacle_count >= hysteresis.static_obstacle_confirm_ticks;

  // Traffic light stop hysteresis.
  const bool tl_stop =
    distance_to_traffic_light <= params.junction_detection_distance &&
    (relevant_traffic_light_state == static_cast<uint8_t>(TrafficLightState::Red) ||
     relevant_traffic_light_state == static_cast<uint8_t>(TrafficLightState::Yellow));

  if (tl_stop) {
    traffic_light_count = std::min(
      traffic_light_count + 1,
      hysteresis.traffic_light_confirm_ticks + hysteresis.exit_hysteresis_ticks);
  } else {
    traffic_light_count = std::max(traffic_light_count - 1, 0);
  }

  stable_traffic_light_stop = traffic_light_count >= hysteresis.traffic_light_confirm_ticks;
}

void BehaviorContext::updateGoalReached()
{
  if (goal_reached || global_route.waypoints.empty()) {
    return;
  }

  const auto & last_wp = global_route.waypoints.back();
  if (distanceToPoint(last_wp.x, last_wp.y) < params.goal_reached_threshold) {
    goal_reached = true;
  }
}

void BehaviorContext::updateLeadingVehicle()
{
  has_leading_vehicle = false;
  leading_vehicle = std::nullopt;

  double best_fwd = std::numeric_limits<double>::max();

  for (const auto & obj : objects) {
    if (!obj.label.empty() && obj.label != "vehicle") {
      continue;
    }

    const double fwd = forwardDistanceToPoint(obj.x, obj.y);
    const double dist = distanceToPoint(obj.x, obj.y);
    const double lat = lateralDistanceToRoute(obj.x, obj.y);

    if (fwd <= 0.0) {
      continue;
    }
    if (dist > params.leading_vehicle_detection_distance) {
      continue;
    }
    if (lat > params.route_lateral_threshold) {
      continue;
    }

    if (fwd < best_fwd) {
      best_fwd = fwd;

      TrackedObject lv = obj;
      lv.forward_distance = fwd;
      lv.route_lateral_distance = lat;

      leading_vehicle = lv;
      has_leading_vehicle = true;
    }
  }
}

void BehaviorContext::updateRelevantTrafficLight()
{
  distance_to_traffic_light = std::numeric_limits<double>::max();
  relevant_traffic_light_state = static_cast<uint8_t>(TrafficLightState::Unknown);

  static constexpr uint8_t kRed = static_cast<uint8_t>(TrafficLightState::Red);
  static constexpr uint8_t kYellow = static_cast<uint8_t>(TrafficLightState::Yellow);
  static constexpr uint8_t kGreen = static_cast<uint8_t>(TrafficLightState::Green);
  static constexpr uint8_t kUnknown = static_cast<uint8_t>(TrafficLightState::Unknown);

  static constexpr double kHeadingToleranceRad = 45.0 * M_PI / 180.0;
  static constexpr double kTlLateralThreshold = 5.0;

  const double ego_yaw = egoYaw();
  const double ex = ego_odom.pose.pose.position.x;
  const double ey = ego_odom.pose.pose.position.y;

  uint8_t worst_state = kUnknown;
  double worst_state_distance = std::numeric_limits<double>::max();

  // Priority of each state when picking which single light is "the relevant one":
  // a closer green light must never override a farther red light that actually
  // controls the ego's lane. Red beats yellow beats green beats unknown, and ties
  // within the same state are broken by distance.
  const auto statePriority = [](uint8_t state) -> int {
    if (state == kRed) {
      return 3;
    }
    if (state == kYellow) {
      return 2;
    }
    if (state == kGreen) {
      return 1;
    }
    return 0;
  };
  int worst_priority = -1;

  for (const auto & tl : traffic_lights) {
    const double fwd = forwardDistanceToPoint(tl.x, tl.y);
    const double dist = distanceToPoint(tl.x, tl.y);

    if (fwd <= 0.0) {
      continue;
    }
    if (dist > params.junction_detection_distance) {
      continue;
    }

    // Reject traffic lights outside the ego heading cone.
    double heading_diff = std::atan2(tl.y - ey, tl.x - ex) - ego_yaw;
    while (heading_diff > M_PI) {
      heading_diff -= 2.0 * M_PI;
    }
    while (heading_diff < -M_PI) {
      heading_diff += 2.0 * M_PI;
    }

    if (std::abs(heading_diff) > kHeadingToleranceRad) {
      continue;
    }

    // Reject traffic lights far from the current route corridor.
    if (lateralDistanceToRoute(tl.x, tl.y) > kTlLateralThreshold) {
      continue;
    }

    const int priority = statePriority(tl.state);
    if (
      priority > worst_priority ||
      (priority == worst_priority && dist < worst_state_distance))
    {
      worst_priority = priority;
      worst_state = tl.state;
      worst_state_distance = dist;
    }
  }

  relevant_traffic_light_state = worst_state;
  if (worst_priority >= 0) {
    distance_to_traffic_light = worst_state_distance;
  }
}

// BehaviorContext: query helpers

double BehaviorContext::egoYaw() const
{
  const auto & q = ego_odom.pose.pose.orientation;
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double BehaviorContext::distanceToPoint(double px, double py) const
{
  return std::hypot(px - ego_odom.pose.pose.position.x, py - ego_odom.pose.pose.position.y);
}

double BehaviorContext::forwardDistanceToPoint(double px, double py) const
{
  const double ex = ego_odom.pose.pose.position.x;
  const double ey = ego_odom.pose.pose.position.y;
  const double yaw = egoYaw();
  return (px - ex) * std::cos(yaw) + (py - ey) * std::sin(yaw);
}

double BehaviorContext::egoSpeed() const
{
  const auto & v = ego_odom.twist.twist.linear;
  return std::hypot(v.x, v.y);
}

double BehaviorContext::lateralDistanceToRoute(double px, double py) const
{
  if (global_route.waypoints.empty()) {
    return std::numeric_limits<double>::max();
  }

  double min_dist = std::numeric_limits<double>::max();
  for (const auto & wp : global_route.waypoints) {
    const double d = std::hypot(px - wp.x, py - wp.y);
    if (d < min_dist) {
      min_dist = d;
    }
  }
  return min_dist;
}

// ConditionView

void ConditionView::setAvoidanceLatched(bool value) { ctx_.avoidance_latched = value; }

void ConditionView::setAvoidancePermanentlyBlocked(bool value)
{
  ctx_.avoidance_permanently_blocked = value;
}

void ConditionView::incrementStallTicks() { ++ctx_.avoidance_stall_ticks; }

void ConditionView::resetStallTicks() { ctx_.avoidance_stall_ticks = 0; }

void ConditionView::setAvoidanceLastFwd(double fwd) { ctx_.avoidance_last_fwd = fwd; }

void ConditionView::latchObstacle(double x, double y, double half_length, double forward_dist)
{
  ctx_.latched_obstacle_x = x;
  ctx_.latched_obstacle_y = y;
  ctx_.latched_obstacle_half_length = half_length;
  ctx_.latched_obstacle_forward_dist = forward_dist;
}

void ConditionView::resetAvoidanceState() { ctx_.resetAvoidanceState(); }

// ActionView

planning_msgs::msg::BehaviorCommand & ActionView::command() { return ctx_.current_command; }

const BehaviorContext::Params & ActionView::params() const { return ctx_.params; }

const std::optional<TrackedObject> & ActionView::leadingVehicle() const
{
  return ctx_.leading_vehicle;
}

double ActionView::distanceToTrafficLight() const { return ctx_.distance_to_traffic_light; }

uint8_t ActionView::relevantTrafficLightState() const { return ctx_.relevant_traffic_light_state; }

}  // namespace behavior_planner
