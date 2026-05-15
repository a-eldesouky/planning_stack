#include "behavior_planner/actions.hpp"

#include <algorithm>
#include <rclcpp/logging.hpp>
#include <utility>

#include "behavior_planner/behavior_context.hpp"

namespace behavior_planner
{

StopAtTrafficLight::StopAtTrafficLight(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ActionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus StopAtTrafficLight::tick()
{
  auto & cmd = view_.command();

  cmd.behavior = static_cast<uint8_t>(BehaviorMode::StopAtTrafficLight);
  cmd.target_speed = 0.0f;
  cmd.safe_distance = static_cast<float>(std::max(view_.distanceToTrafficLight(), 1.0));
  cmd.reason = reason::kStopAtTrafficLight;

  RCLCPP_INFO_THROTTLE(
    logger_, *clock_, 2000, "[%s] TL dist=%.2f m state=%u",
    behaviorModeToString(BehaviorMode::StopAtTrafficLight), view_.distanceToTrafficLight(),
    view_.relevantTrafficLightState());

  return BT::NodeStatus::SUCCESS;
}

RequestAvoidance::RequestAvoidance(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ActionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus RequestAvoidance::tick()
{
  auto & cmd = view_.command();

  cmd.behavior = static_cast<uint8_t>(BehaviorMode::AvoidStaticObstacle);
  cmd.target_speed = static_cast<float>(view_.params().avoidance_speed);
  cmd.safe_distance = static_cast<float>(view_.params().safe_following_distance);
  cmd.reason = reason::kAvoidStaticObstacle;

  if (view_.leadingVehicle().has_value()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[%s] obstacle at (%.1f, %.1f) fwd=%.2f m -> speed=%.1f m/s",
      behaviorModeToString(BehaviorMode::AvoidStaticObstacle), view_.leadingVehicle()->x,
      view_.leadingVehicle()->y, view_.leadingVehicle()->forward_distance,
      view_.params().avoidance_speed);
  }

  return BT::NodeStatus::SUCCESS;
}

WaitBehindObstacle::WaitBehindObstacle(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ActionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus WaitBehindObstacle::tick()
{
  auto & cmd = view_.command();

  cmd.behavior = static_cast<uint8_t>(BehaviorMode::WaitBehindObstacle);
  cmd.target_speed = 0.0f;
  cmd.safe_distance = static_cast<float>(view_.params().safe_following_distance);
  cmd.reason = reason::kWaitBehindObstacle;

  if (view_.leadingVehicle().has_value()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[%s] Holding. obstacle fwd=%.2f m",
      behaviorModeToString(BehaviorMode::WaitBehindObstacle),
      view_.leadingVehicle()->forward_distance);
  }

  return BT::NodeStatus::SUCCESS;
}

FollowLeadingVehicle::FollowLeadingVehicle(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ActionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus FollowLeadingVehicle::tick()
{
  double target_speed = view_.params().following_speed;

  if (view_.leadingVehicle().has_value()) {
    target_speed = std::min(target_speed, view_.leadingVehicle()->speed);
    target_speed = std::max(target_speed, 0.0);
  }

  auto & cmd = view_.command();

  cmd.behavior = static_cast<uint8_t>(BehaviorMode::FollowLeadingVehicle);
  cmd.target_speed = static_cast<float>(target_speed);
  cmd.safe_distance = static_cast<float>(view_.params().safe_following_distance);
  cmd.reason = reason::kFollowLeadingVehicle;

  if (view_.leadingVehicle().has_value()) {
    RCLCPP_INFO_THROTTLE(
      logger_, *clock_, 2000, "[%s] leader speed=%.2f m/s fwd=%.2f m -> target=%.2f m/s",
      behaviorModeToString(BehaviorMode::FollowLeadingVehicle), view_.leadingVehicle()->speed,
      view_.leadingVehicle()->forward_distance, target_speed);
  }

  return BT::NodeStatus::SUCCESS;
}

FollowLane::FollowLane(
  const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
  rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
: ActionBase(name, config, std::move(ctx), logger, std::move(clock))
{
}

BT::NodeStatus FollowLane::tick()
{
  auto & cmd = view_.command();

  cmd.behavior = static_cast<uint8_t>(BehaviorMode::FollowLane);
  cmd.target_speed = static_cast<float>(view_.params().normal_speed);
  cmd.safe_distance = static_cast<float>(view_.params().safe_following_distance);
  cmd.reason = reason::kFollowLane;

  RCLCPP_DEBUG(
    logger_, "[%s] %.2f m/s", behaviorModeToString(BehaviorMode::FollowLane),
    view_.params().normal_speed);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace behavior_planner
