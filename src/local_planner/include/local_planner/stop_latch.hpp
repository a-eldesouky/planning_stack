#ifndef LOCAL_PLANNER__STOP_LATCH_HPP_
#define LOCAL_PLANNER__STOP_LATCH_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "local_planner/planning_types.hpp"

namespace local_planner
{

/**
 * @class StopLatch
 * @brief Manages the "committed stop point" state machine.
 *
 * When a stopping behavior (STOP_AT_TRAFFIC_LIGHT, WAIT_BEHIND_OBSTACLE) brings
 * the ego within `commit_distance` of the planned stop point, the stop location
 * is latched. The latch prevents a one-cycle behavior dropout from
 * re-accelerating the vehicle before the light turns green or the obstacle clears.
 *
 * ## Owned state
 * - `committed_`   : latch flag.
 * - `stop_x/y_`    : fixed map-frame stop location.
 * - `non_stopping_ticks_`: consecutive ticks where the behavior is no longer stopping.
 *
 * ## Release conditions (both must be satisfied)
 * 1. `kRequiredClearTicks` consecutive non-stopping ticks, AND
 * 2. Ego is stopped (speed ≤ 0.20 m/s) within `commit_distance`.
 *    OR the ego has already passed the committed stop point.
 */
class StopLatch
{
public:
  /// Minimum consecutive non-stopping ticks before the latch can be released.
  static constexpr int kRequiredClearTicks{30};

  /**
   * @param commit_distance Distance to the stop point that arms the latch [m].
   * @param creep_speed     Speed cap applied in the creep zone [m/s].
   */
  StopLatch(double commit_distance, double creep_speed);

  // Mutators

  /**
   * @brief Arm the latch at the given stop-point location.
   * @param stop_x Stop point x in map frame [m].
   * @param stop_y Stop point y in map frame [m].
   * @param reason Reason string recorded on the final creep-trajectory point.
   */
  void commit(double stop_x, double stop_y, std::string reason);

  // Query / update

  /// True when a stop point is currently committed.
  bool isCommitted() const { return committed_; }

  /**
   * @brief Evaluate and act on the committed stop.
   *
   * Must be called every planning tick while `isCommitted()` is true.
   * Writes a creep-to-stop trajectory into `trajectory_out` and returns true
   * if the latch is still active, or releases the latch and returns false
   * when the release conditions are met.
   *
   * @param ego              Current ego pose.
   * @param currently_stopping True if the current behavior is a stop behavior.
   * @param ego_speed        Current ego speed [m/s].
   * @param trajectory_out   Output trajectory (modified only when returning true).
   * @param clock            Node clock (for throttled log).
   * @param logger           ROS logger for diagnostics.
   * @return True if the latch is still holding (caller should return early).
   */
  [[nodiscard]] bool update(
    const Pose & ego, bool currently_stopping, double ego_speed,
    std::vector<TrajectoryPoint> & trajectory_out, rclcpp::Clock & clock,
    const rclcpp::Logger & logger);

  /**
   * @brief Maybe arm the latch if the stop point is within commit_distance.
   *
   * Called after a stopping trajectory has been built. Inspects the trajectory's
   * last point and commits if the ego is close enough.
   *
   * @param ego          Current ego pose.
   * @param trajectory   The current stopping trajectory.
   * @param logger       ROS logger.
   * @param reason       Reason string to record on commit (e.g. "traffic_light_committed").
   */
  void maybeCommit(
    const Pose & ego, const std::vector<TrajectoryPoint> & trajectory,
    const rclcpp::Logger & logger, std::string reason);

private:
  double commit_distance_;
  double creep_speed_;

  bool committed_{false};
  double stop_x_{0.0};
  double stop_y_{0.0};
  std::string stop_reason_{"stop_committed"};
  int non_stopping_ticks_{0};
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__STOP_LATCH_HPP_
