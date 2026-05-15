#include "behavior_planner/conditions.hpp"

#include <rclcpp/logging.hpp>
#include <utility>

namespace behavior_planner
{

IsAtTJunction::IsAtTJunction(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsAtTJunction::tick()
{
  const auto & c = view_.ctx();

  if (c.traffic_lights.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  if (c.distance_to_traffic_light > c.params.junction_detection_distance) {
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000, "[IsAtTJunction] TL dist=%.2f m threshold=%.2f m",
    c.distance_to_traffic_light, c.params.junction_detection_distance);

  return BT::NodeStatus::SUCCESS;
}

IsTrafficLightRedOrYellow::IsTrafficLightRedOrYellow(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsTrafficLightRedOrYellow::tick()
{
  const auto & c = view_.ctx();

  if (c.stable_traffic_light_stop) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[IsTrafficLightRedOrYellow] Stable RED/YELLOW (state=%u, count=%d) - stopping.",
      c.relevant_traffic_light_state, c.traffic_light_count);

    return BT::NodeStatus::SUCCESS;
  }

  if (c.relevant_traffic_light_state == 0 || c.relevant_traffic_light_state == 1) {
    RCLCPP_DEBUG(
      logger_, "[IsTrafficLightRedOrYellow] RED/YELLOW detected - confirming (%d/%d ticks)...",
      c.traffic_light_count, c.hysteresis.traffic_light_confirm_ticks);
  }

  return BT::NodeStatus::FAILURE;
}

IsLeadingVehicleDetected::IsLeadingVehicleDetected(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsLeadingVehicleDetected::tick()
{
  const auto & c = view_.ctx();

  if (c.avoidance_latched) {
    RCLCPP_DEBUG(logger_, "[IsLeadingVehicleDetected] Avoidance latched: holding detection");
    return BT::NodeStatus::SUCCESS;
  }

  if (!c.stable_leading_vehicle_detected) {
    view_.resetAvoidanceState();
    return BT::NodeStatus::FAILURE;
  }

  if (!c.leading_vehicle.has_value()) {
    view_.resetAvoidanceState();
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(
    logger_, "[IsLeadingVehicleDetected] id=%u fwd=%.2f m speed=%.2f m/s", c.leading_vehicle->id,
    c.leading_vehicle->forward_distance, c.leading_vehicle->speed);

  return BT::NodeStatus::SUCCESS;
}

IsLeadingVehicleStatic::IsLeadingVehicleStatic(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsLeadingVehicleStatic::tick()
{
  const auto & c = view_.ctx();

  if (c.avoidance_latched) {
    return BT::NodeStatus::SUCCESS;
  }

  if (!c.leading_vehicle.has_value()) {
    return BT::NodeStatus::FAILURE;
  }

  if (!c.stable_static_obstacle) {
    RCLCPP_DEBUG(
      logger_, "[IsLeadingVehicleStatic] Confirming static... %d/%d ticks, speed=%.2f m/s",
      c.static_obstacle_count, c.hysteresis.static_obstacle_confirm_ticks,
      c.leading_vehicle->speed);

    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_DEBUG(
    logger_, "[IsLeadingVehicleStatic] Confirmed STATIC (speed=%.2f m/s)",
    c.leading_vehicle->speed);

  return BT::NodeStatus::SUCCESS;
}

IsLeadingVehicleMoving::IsLeadingVehicleMoving(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsLeadingVehicleMoving::tick()
{
  const auto & c = view_.ctx();

  if (!c.leading_vehicle.has_value()) {
    return BT::NodeStatus::FAILURE;
  }

  const bool is_moving = c.leading_vehicle->speed >= c.params.static_speed_threshold;

  RCLCPP_DEBUG(
    logger_, "[IsLeadingVehicleMoving] speed=%.2f -> %s", c.leading_vehicle->speed,
    is_moving ? "MOVING" : "STATIC");

  return is_moving ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

IsAvoidancePossible::IsAvoidancePossible(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ConditionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus IsAvoidancePossible::tick()
{
  const auto & c = view_.ctx();

  if (!c.params.enable_avoidance) {
    view_.setAvoidanceLatched(false);
    RCLCPP_DEBUG(logger_, "[IsAvoidancePossible] Disabled by parameter.");
    return BT::NodeStatus::FAILURE;
  }

  if (c.avoidance_latched) {
    const double fwd = c.forwardDistanceToPoint(c.latched_obstacle_x, c.latched_obstacle_y);

    const double ego_half_length = 1.15;
    const double safety_margin = 2.0;
    const double release_threshold =
      -(c.latched_obstacle_half_length + ego_half_length + safety_margin);

    if (fwd < release_threshold) {
      view_.setAvoidancePermanentlyBlocked(false);
      view_.setAvoidanceLatched(false);
      RCLCPP_INFO(
        logger_,
        "[IsAvoidancePossible] Obstacle cleared: fwd=%.2f m < threshold=%.2f m "
        "(half_len=%.2f m): releasing latch",
        fwd, release_threshold, c.latched_obstacle_half_length);
      return BT::NodeStatus::FAILURE;
    }

    if (std::abs(fwd - c.avoidance_last_fwd) < 0.05) {
      view_.incrementStallTicks();
    } else {
      view_.resetStallTicks();
    }
    view_.setAvoidanceLastFwd(fwd);

    if (c.avoidance_stall_ticks >= BehaviorContext::kAvoidStallLimit) {
      view_.setAvoidanceLatched(false);
      view_.setAvoidancePermanentlyBlocked(true);
      view_.resetStallTicks();
      RCLCPP_WARN(
        logger_,
        "[IsAvoidancePossible] Avoidance stalled for %d ticks (fwd=%.2f m): "
        "forcing WAIT_BEHIND_OBSTACLE",
        BehaviorContext::kAvoidStallLimit, fwd);
      return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(
      logger_, "[IsAvoidancePossible] LATCHED: fwd=%.2f m, release at %.2f m (stall %d/%d)", fwd,
      release_threshold, c.avoidance_stall_ticks, BehaviorContext::kAvoidStallLimit);
    return BT::NodeStatus::SUCCESS;
  }

  if (!c.leading_vehicle.has_value()) {
    return BT::NodeStatus::FAILURE;
  }

  const double fwd = c.leading_vehicle->forward_distance;

  const double ego_half_length = 1.15;
  const double obs_half_length = c.leading_vehicle->half_length;
  const double obs_circle_radius = 1.0 + 0.3;
  const double min_safe_fwd = obs_half_length + obs_circle_radius + ego_half_length + 0.5;

  if (c.avoidance_permanently_blocked) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000,
      "[IsAvoidancePossible] Avoidance blocked for this obstacle: forcing WAIT");
    return BT::NodeStatus::FAILURE;
  }

  const bool possible = fwd > c.params.minimum_avoidance_distance && fwd > min_safe_fwd;

  RCLCPP_DEBUG(
    logger_, "[IsAvoidancePossible] fwd=%.2f m runway_min=%.2f m safe_min=%.2f m -> %s", fwd,
    c.params.minimum_avoidance_distance, min_safe_fwd, possible ? "POSSIBLE" : "NO RUNWAY");

  if (possible) {
    view_.setAvoidanceLatched(true);
    view_.resetStallTicks();
    view_.setAvoidanceLastFwd(fwd);
    view_.latchObstacle(c.leading_vehicle->x, c.leading_vehicle->y, obs_half_length, fwd);
    RCLCPP_INFO(
      logger_,
      "[IsAvoidancePossible] Latch ARMED: fwd=%.2f m (safe_min=%.2f m), "
      "half_len=%.2f m, obstacle at (%.2f, %.2f), release at fwd < %.2f m",
      fwd, min_safe_fwd, obs_half_length, c.leading_vehicle->x, c.leading_vehicle->y,
      -(obs_half_length + ego_half_length + 2.0));
  }

  return possible ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace behavior_planner
