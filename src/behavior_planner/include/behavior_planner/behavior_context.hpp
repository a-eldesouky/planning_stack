#ifndef BEHAVIOR_PLANNER__BEHAVIOR_CONTEXT_HPP_
#define BEHAVIOR_PLANNER__BEHAVIOR_CONTEXT_HPP_

#include <cstdint>
#include <limits>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <string>
#include <vector>

#include "planning_msgs/msg/behavior_command.hpp"
#include "planning_msgs/msg/route.hpp"

namespace behavior_planner
{

/**
 * @brief High-level behavior modes selected by the behavior tree.
 */
enum class BehaviorMode : uint8_t
{
  FollowLane = 0,
  FollowLeadingVehicle = 1,
  AvoidStaticObstacle = 2,
  StopAtTrafficLight = 3,
  WaitBehindObstacle = 4
};

/**
 * @brief CARLA traffic light states.
 *
 * Matches the integer values published by the CARLA ROS bridge on
 * /carla/traffic_lights/status (CarlaTrafficLightStatusList).
 */
enum class TrafficLightState : uint8_t
{
  Red = 0,
  Yellow = 1,
  Green = 2,
  Unknown = 3,
};

/// Convert a BehaviorMode to a human-readable string.
const char * behaviorModeToString(BehaviorMode mode);

namespace reason
{
constexpr const char * kFollowLane = "No higher-priority behavior active";
constexpr const char * kFollowLeadingVehicle = "Moving leading vehicle ahead";
constexpr const char * kAvoidStaticObstacle =
  "Static obstacle on route - avoidance maneuver requested";
constexpr const char * kStopAtTrafficLight = "Red/yellow traffic light at junction";
constexpr const char * kWaitBehindObstacle = "Static obstacle ahead - no safe avoidance path";
}  // namespace reason

/**
 * @brief Simplified tracked object used by the behavior planner.
 */
struct TrackedObject
{
  uint32_t id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double speed{0.0};
  std::string label;
  double forward_distance{0.0};
  double route_lateral_distance{0.0};
  double half_length{2.25};  ///< Half-length along travel axis [m]
};

/**
 * @brief Fused traffic light entry used by the behavior planner.
 */
struct TrafficLightEntry
{
  uint32_t id{0};
  uint8_t state{static_cast<uint8_t>(TrafficLightState::Unknown)};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/**
 * @class BehaviorContext
 * @brief Shared behavior planner state, helper queries, and update methods.
 *
 * Acts as the behavior tree blackboard:
 * - ROS callbacks (owned by BehaviorPlannerNode) write raw sensor/route data.
 * - `update*()` methods derive stable flags from that raw data each tick.
 * - Condition nodes read via ConditionView; action nodes write via ActionView.
 * - The node publishes `current_command` after each BT tick.
 *
 * @note The mutex protects the context when callbacks and the BT tick run
 *       concurrently.  Helper methods do NOT lock internally.
 */
class BehaviorContext
{
public:
  struct Params
  {
    double normal_speed{8.0};
    double following_speed{4.0};
    double avoidance_speed{3.0};
    double leading_vehicle_detection_distance{30.0};
    double safe_following_distance{10.0};
    double route_lateral_threshold{3.5};
    double static_speed_threshold{0.5};
    double ego_filter_radius{2.5};
    double junction_detection_distance{15.0};
    double traffic_light_stop_distance{12.0};
    bool enable_avoidance{true};
    double minimum_avoidance_distance{5.0};
    double road_boundary_margin{0.5};
    double goal_reached_threshold{2.0};
  };

  struct HysteresisParams
  {
    int leading_vehicle_confirm_ticks{3};
    int static_obstacle_confirm_ticks{5};
    int traffic_light_confirm_ticks{2};
    int exit_hysteresis_ticks{5};
  };

  /// BT ticks in AVOID_STATIC_OBSTACLE with no forward progress before giving up.
  static constexpr int kAvoidStallLimit{30};

  Params params;
  nav_msgs::msg::Odometry ego_odom;
  bool ego_odom_received{false};
  planning_msgs::msg::Route global_route;
  bool route_received{false};
  std::vector<TrackedObject> objects;
  std::vector<TrafficLightEntry> traffic_lights;
  HysteresisParams hysteresis;

  bool goal_reached{false};
  std::optional<TrackedObject> leading_vehicle;
  bool has_leading_vehicle{false};
  double distance_to_traffic_light{std::numeric_limits<double>::max()};
  uint8_t relevant_traffic_light_state{static_cast<uint8_t>(TrafficLightState::Unknown)};

  int leading_vehicle_count{0};
  int static_obstacle_count{0};
  int traffic_light_count{0};

  bool stable_leading_vehicle_detected{false};
  bool stable_static_obstacle{false};
  bool stable_traffic_light_stop{false};

  bool avoidance_latched{false};
  bool avoidance_permanently_blocked{false};
  int avoidance_stall_ticks{0};
  double avoidance_last_fwd{0.0};
  double latched_obstacle_half_length{2.25};
  double latched_obstacle_x{0.0};
  double latched_obstacle_y{0.0};
  double latched_obstacle_forward_dist{0.0};

  planning_msgs::msg::BehaviorCommand current_command;

  mutable std::mutex mtx;

  /** @brief Reset all avoidance latch fields to their default values. */
  void resetAvoidanceState();

  /** @brief Compute stable detection flags from raw counters. */
  void updateHysteresis();

  /** @brief Update goal_reached from distance to last route waypoint. */
  void updateGoalReached();

  /** @brief Select the closest route-relevant leading vehicle from objects. */
  void updateLeadingVehicle();

  /**
   * @brief Select the relevant traffic light and its worst-case state.
   *
   * Filters by: ahead of ego, within junction_detection_distance, heading
   * cone alignment, and lateral proximity to route.
   */
  void updateRelevantTrafficLight();

  // Query helpers (called by update methods and views, definitions in .cpp)

  /** @brief Ego yaw in radians extracted from odometry quaternion. */
  double egoYaw() const;

  /** @brief 2D Euclidean distance from ego to world point (px, py) [m]. */
  double distanceToPoint(double px, double py) const;

  /**
   * @brief Signed longitudinal distance from ego to world point (px, py) [m].
   *
   * Positive = point is ahead of ego, negative = behind.
   */
  double forwardDistanceToPoint(double px, double py) const;

  /** @brief Ego speed magnitude from odometry twist [m/s]. */
  double egoSpeed() const;

  /**
   * @brief Minimum 2D distance from world point (px, py) to any route waypoint [m].
   *
   * Returns max double when the route is empty.
   */
  double lateralDistanceToRoute(double px, double py) const;
};

/**
 * @brief Narrow interface for condition nodes: read-only context + avoidance writes.
 *
 * Passing this view instead of a raw shared_ptr<BehaviorContext> prevents
 * conditions from accidentally writing fields they should not own
 * (e.g. current_command, ego_odom, objects).
 *
 * Write access is limited to the avoidance latch fields through named mutators.
 * All other context fields are accessible as const via ctx().
 */
class ConditionView
{
public:
  explicit ConditionView(BehaviorContext & ctx)
  : ctx_(ctx) {}

  // Read-only
  const BehaviorContext & ctx() const { return ctx_; }

  // Avoidance latch mutators
  void setAvoidanceLatched(bool value);
  void setAvoidancePermanentlyBlocked(bool value);
  void incrementStallTicks();
  void resetStallTicks();
  void setAvoidanceLastFwd(double fwd);
  void latchObstacle(double x, double y, double half_length, double forward_dist);
  void resetAvoidanceState();

private:
  BehaviorContext & ctx_;
};

/**
 * @brief Narrow interface for action nodes: write current_command + read inputs.
 *
 * Actions can only write current_command and read the small set of context
 * fields needed to populate it.  All other fields are inaccessible.
 */
class ActionView
{
public:
  explicit ActionView(BehaviorContext & ctx)
  : ctx_(ctx) {}

  // Write surface
  planning_msgs::msg::BehaviorCommand & command();

  // Read surface
  const BehaviorContext::Params & params() const;
  const std::optional<TrackedObject> & leadingVehicle() const;
  double distanceToTrafficLight() const;
  uint8_t relevantTrafficLightState() const;

private:
  BehaviorContext & ctx_;
};

}  // namespace behavior_planner

#endif  // BEHAVIOR_PLANNER__BEHAVIOR_CONTEXT_HPP_
