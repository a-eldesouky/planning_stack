#ifndef BEHAVIOR_PLANNER__ACTIONS_HPP_
#define BEHAVIOR_PLANNER__ACTIONS_HPP_

#include <behaviortree_cpp/behavior_tree.h>

#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behavior_planner/behavior_context.hpp"

namespace behavior_planner
{

/**
 * @brief Base class for synchronous behavior tree action nodes.
 *
 * This class stores the shared behavior context, ROS logger, and clock used by
 * all behavior planner action nodes. It avoids duplicating common constructor
 * arguments across the concrete action implementations.
 */
class ActionBase : public BT::SyncActionNode
{
public:
  /**
   * @brief Construct a behavior tree action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context containing planner state and decisions.
   * @param logger ROS 2 logger used by the action node.
   * @param clock ROS 2 clock used for throttled logging or timing logic.
   */
  ActionBase(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : BT::SyncActionNode(name, config),
    ctx_(std::move(ctx)),
    view_(*ctx_),
    logger_(logger),
    clock_(std::move(clock))
  {
  }

  /**
   * @brief Return the ports exposed by this node.
   *
   * The current action nodes use the shared behavior context directly and do not
   * expose BehaviorTree.CPP input or output ports.
   *
   * @return Empty port list.
   */
  static BT::PortsList providedPorts() { return {}; }

protected:
  std::shared_ptr<BehaviorContext> ctx_;  ///< Kept alive; use view_ in tick() implementations.
  ActionView view_;                       ///< Narrow interface: write current_command, read inputs.
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

/**
 * @brief Action node that commands the vehicle to stop for a traffic light.
 *
 * This action is expected to be selected when the behavior tree detects a red or
 * blocking traffic light ahead. It updates the behavior context with a stop
 * command for downstream planning/control modules.
 */
class StopAtTrafficLight : public ActionBase
{
public:
  /**
   * @brief Construct the StopAtTrafficLight action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  StopAtTrafficLight(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Execute the stop-at-traffic-light behavior.
   *
   * @return BT::NodeStatus::SUCCESS when the behavior command is written.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Action node that requests an obstacle avoidance maneuver.
 *
 * This action is used when a static obstacle blocks the current lane and the
 * behavior planner decides that an avoidance maneuver should be attempted by
 * the downstream local planner.
 */
class RequestAvoidance : public ActionBase
{
public:
  /**
   * @brief Construct the RequestAvoidance action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  RequestAvoidance(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Execute the avoidance request behavior.
   *
   * @return BT::NodeStatus::SUCCESS when the avoidance command is written.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Action node that commands the vehicle to wait behind an obstacle.
 *
 * This action is used when a leading obstacle is blocking the lane and no safe
 * avoidance behavior should be requested.
 */
class WaitBehindObstacle : public ActionBase
{
public:
  /**
   * @brief Construct the WaitBehindObstacle action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  WaitBehindObstacle(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Execute the wait-behind-obstacle behavior.
   *
   * @return BT::NodeStatus::SUCCESS when the wait command is written.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Action node that commands vehicle-following behavior.
 *
 * This action is used when a moving leading vehicle is detected ahead. It
 * requests a following behavior instead of free lane following.
 */
class FollowLeadingVehicle : public ActionBase
{
public:
  /**
   * @brief Construct the FollowLeadingVehicle action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  FollowLeadingVehicle(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Execute the follow-leading-vehicle behavior.
   *
   * @return BT::NodeStatus::SUCCESS when the following command is written.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Default action node for normal lane-following behavior.
 *
 * This action is selected when no higher-priority behavior is active, such as
 * traffic-light stopping, obstacle avoidance, or leading-vehicle following.
 */
class FollowLane : public ActionBase
{
public:
  /**
   * @brief Construct the FollowLane action node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  FollowLane(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Execute the default lane-following behavior.
   *
   * @return BT::NodeStatus::SUCCESS when the lane-following command is written.
   */
  BT::NodeStatus tick() override;
};

}  // namespace behavior_planner

#endif  // BEHAVIOR_PLANNER__ACTIONS_HPP_
