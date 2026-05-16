#ifndef LOCAL_PLANNER__AVOIDANCE_LATCH_HPP_
#define LOCAL_PLANNER__AVOIDANCE_LATCH_HPP_

#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "local_planner/planning_types.hpp"

namespace local_planner
{

/**
 * @class AvoidanceLatch
 * @brief Manages the "committed avoidance path" state machine.
 *
 * Once Hybrid A* finds a valid bypass, the result is latched here and followed
 * until the obstacle is confirmed passed. This prevents the planner from
 * re-planning from inside the obstacle footprint while the ego is beside it.
 *
 * ## Owned state
 * - `latched_path_`          : the cached Hybrid A* trajectory.
 * - `latched_obstacle_`      : the farthest obstacle circle at latch time.
 * - `consecutive_failures_`  : A* failure counter; resets the latch after
 *                               kMaxFailures to allow fallback to WAIT_BEHIND.
 *
 * ## Lifecycle
 * 1. `reset()`: on new route or behavior clearing avoidance.
 * 2. `latch(path, obstacles, ego)`: A* succeeded; store path and obstacle.
 * 3. `recordFailure()`: A* failed; increment counter, auto-clear at limit.
 * 4. `isLatched()` / `latchedPath()` / `isObstacleCleared(ego)`: query state.
 * 5. `trimFromEgo(path, ego)`: return the remaining segment of the latched path.
 */
class AvoidanceLatch
{
public:
  /// Maximum consecutive A* failures before the latch is auto-released.
  static constexpr int kMaxFailures{20};

  /**
   * @brief Construct with the obstacle release distance.
   * @param release_distance Distance behind the ego before releasing [m].
   */
  explicit AvoidanceLatch(double release_distance);

  /// Reset all latch state (new route, behavior cleared).
  void reset();

  /**
   * @brief Commit to a new avoidance path.
   *
   * Stores the path and picks the farthest-forward obstacle circle as the
   * release reference (farthest = ego must fully pass the front of the obstacle).
   *
   * @param path      Hybrid A* result trajectory.
   * @param obstacles Current obstacle list at planning time.
   * @param ego       Current ego pose (used to project "farthest forward").
   */
  void latch(
    const std::vector<TrajectoryPoint> & path, const std::vector<Obstacle> & obstacles,
    const Pose & ego);

  /**
   * @brief Record one A* planning failure.
   *
   * Auto-resets the latch when kMaxFailures is reached so the behavior
   * tree can fall through to WAIT_BEHIND_OBSTACLE.
   *
   * @param logger ROS logger used to emit a warning on auto-reset.
   * @return True if the failure limit was reached and the latch was cleared.
   */
  bool recordFailure(const rclcpp::Logger & logger);

  /// Reset only the failure counter (called after a successful plan).
  void resetFailureCount() { consecutive_failures_ = 0; }

  /// True while a committed avoidance path is held.
  bool isLatched() const { return latched_; }

  /// Failure counter value (for diagnostics).
  int consecutiveFailures() const { return consecutive_failures_; }

  /// The cached avoidance trajectory (valid only while isLatched()).
  const std::vector<TrajectoryPoint> & latchedPath() const { return latched_path_; }

  /**
   * @brief True once the latched obstacle circle is release_distance_ behind the ego.
   *
   * Uses the FIXED position stored at latch time, not the live obstacle list,
   * so the release is not affected by obstacles rotating in/out of the lookahead
   * window as the ego moves.
   *
   * @param ego Current ego pose.
   */
  bool isObstacleCleared(const Pose & ego) const;

  /**
   * @brief Return the remaining section of the latched path ahead of the ego.
   *
   * Two-pass search: find the closest point by Euclidean distance, then walk
   * forward until the first point with a positive longitudinal projection.
   * If all remaining points are behind the ego, extends the path forward along
   * the last point's heading so the controller always has a valid ahead target.
   *
   * @param ego Current ego pose.
   * @return Trimmed (or extended) trajectory.
   */
  std::vector<TrajectoryPoint> trimFromEgo(const Pose & ego) const;

private:
  double release_distance_;
  bool latched_{false};
  int consecutive_failures_{0};
  std::vector<TrajectoryPoint> latched_path_;
  std::optional<Obstacle> latched_obstacle_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__AVOIDANCE_LATCH_HPP_
