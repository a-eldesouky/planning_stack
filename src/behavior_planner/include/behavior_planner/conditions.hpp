#ifndef BEHAVIOR_PLANNER__CONDITIONS_HPP_
#define BEHAVIOR_PLANNER__CONDITIONS_HPP_

#include <behaviortree_cpp/behavior_tree.h>

#include <memory>
#include <rclcpp/clock.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "behavior_planner/behavior_context.hpp"

namespace behavior_planner
{

/**
 * @brief Base class for synchronous behavior tree condition nodes.
 *
 * This class stores the shared behavior context, ROS logger, and ROS clock used
 * by all behavior planner condition nodes. Concrete conditions read from the
 * shared context and return SUCCESS or FAILURE based on the current driving
 * situation.
 */
class ConditionBase : public BT::ConditionNode
{
public:
  /**
   * @brief Construct a behavior tree condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context containing route, perception, and
   * traffic state.
   * @param logger ROS 2 logger used by the condition node.
   * @param clock ROS 2 clock used for throttled logging and timing logic.
   */
  ConditionBase(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock)
  : BT::ConditionNode(name, config),
    ctx_(std::move(ctx)),
    view_(*ctx_),
    logger_(logger),
    clock_(std::move(clock))
  {
  }

  /**
   * @brief Return the BehaviorTree.CPP ports exposed by this node.
   *
   * The current condition nodes read directly from the shared behavior context
   * and do not require explicit input/output ports.
   *
   * @return Empty port list.
   */
  static BT::PortsList providedPorts() { return {}; }

protected:
  std::shared_ptr<BehaviorContext> ctx_;  ///< Kept alive; use view_ in tick() implementations.
  ConditionView view_;  ///< Narrow interface: read ctx, write only avoidance state.
  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

/**
 * @brief Condition node that checks whether the ego vehicle is near a
 * controlled junction.
 *
 * Controlled T-junction handling is detected using the
 * nearest relevant traffic light distance.
 */
class IsAtTJunction : public ConditionBase
{
public:
  /**
   * @brief Construct the IsAtTJunction condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsAtTJunction(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate whether the ego vehicle is near a controlled junction.
   *
   * @return BT::NodeStatus::SUCCESS if the ego vehicle is near a controlled
   * junction, otherwise BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks whether the relevant traffic light is red
 * or yellow.
 *
 * This condition is used to trigger stopping behavior at controlled junctions.
 */
class IsTrafficLightRedOrYellow : public ConditionBase
{
public:
  /**
   * @brief Construct the IsTrafficLightRedOrYellow condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsTrafficLightRedOrYellow(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate the current relevant traffic light state.
   *
   * @return BT::NodeStatus::SUCCESS for red/yellow traffic light states,
   * otherwise BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks whether a leading vehicle or obstacle is
 * detected.
 *
 * The detected object can later be classified as moving or static by other
 * condition nodes.
 */
class IsLeadingVehicleDetected : public ConditionBase
{
public:
  /**
   * @brief Construct the IsLeadingVehicleDetected condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsLeadingVehicleDetected(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate whether a leading object exists in front of the ego
   * vehicle.
   *
   * @return BT::NodeStatus::SUCCESS if a leading object is detected, otherwise
   * BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks whether the detected leading object is
 * static.
 *
 * A static leading object can trigger either avoidance or waiting behavior.
 */
class IsLeadingVehicleStatic : public ConditionBase
{
public:
  /**
   * @brief Construct the IsLeadingVehicleStatic condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsLeadingVehicleStatic(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate whether the detected leading object is static.
   *
   * @return BT::NodeStatus::SUCCESS if the leading object is static, otherwise
   * BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks whether the detected leading vehicle is
 * moving.
 *
 * A moving leading vehicle triggers vehicle-following behavior.
 */
class IsLeadingVehicleMoving : public ConditionBase
{
public:
  /**
   * @brief Construct the IsLeadingVehicleMoving condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsLeadingVehicleMoving(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate whether the detected leading object is moving.
   *
   * @return BT::NodeStatus::SUCCESS if the leading object is moving, otherwise
   * BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

/**
 * @brief Condition node that checks whether obstacle avoidance is currently
 * allowed.
 *
 * This condition represents a high-level feasibility gate for avoidance. The
 * actual trajectory generation is handled by the local planner.
 */
class IsAvoidancePossible : public ConditionBase
{
public:
  /**
   * @brief Construct the IsAvoidancePossible condition node.
   *
   * @param name BehaviorTree.CPP node name.
   * @param config BehaviorTree.CPP node configuration.
   * @param ctx Shared behavior context.
   * @param logger ROS 2 logger.
   * @param clock ROS 2 clock.
   */
  IsAvoidancePossible(
    const std::string & name, const BT::NodeConfig & config, std::shared_ptr<BehaviorContext> ctx,
    rclcpp::Logger logger, rclcpp::Clock::SharedPtr clock);

  /**
   * @brief Evaluate whether an avoidance maneuver may be requested.
   *
   * @return BT::NodeStatus::SUCCESS if avoidance is allowed, otherwise
   * BT::NodeStatus::FAILURE.
   */
  BT::NodeStatus tick() override;
};

}  // namespace behavior_planner

#endif  // BEHAVIOR_PLANNER__CONDITIONS_HPP_
