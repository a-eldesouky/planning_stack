#ifndef LOCAL_PLANNER__OBSTACLE_PROCESSOR_HPP_
#define LOCAL_PLANNER__OBSTACLE_PROCESSOR_HPP_

#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "local_planner/planning_types.hpp"

namespace local_planner
{

/**
 * @brief Configuration for ObstacleProcessor.
 */
struct ObstacleProcessorParams
{
  double lookahead{30.0};  ///< Forward range to keep obstacles [m]
  double inflation{0.8};   ///< Added to each circle radius [m]
  double corridor_width_multiplier{1.5};
  double avoidance_corridor_multiplier{3.0};
  double min_avoidance_corridor_half_width{5.2};
  double ego_filter_multiplier{0.5};  ///< Self-detection exclusion zone = length * this
};

/**
 * @class ObstacleProcessor
 * @brief Converts raw CARLA object messages into inflated obstacle circles.
 *
 * Owned state: the `obstacles_` vector and all obstacle-detection parameters.
 * The node calls `update()` on every objects callback and reads `obstacles()`
 * when building the avoidance trajectory.
 *
 * ## Design notes
 * - Ego-vehicle filtering is done by Euclidean distance (ego_filter_radius).
 * - Each detected object is approximated by up to 3 circles aligned with the
 *   ego heading (see implementation for why obstacle yaw is not trusted).
 * - `obstacleAhead()` projects filtered circles into the ego frame to confirm
 *   that at least one is within the configured lateral corridor.
 */
class ObstacleProcessor
{
public:
  /**
   * @brief Construct with shared parameters.
   * @param params Detection and inflation settings.
   * @param vehicle Vehicle geometry (length and width used for filtering).
   */
  ObstacleProcessor(ObstacleProcessorParams params, VehicleParams vehicle);

  /**
   * @brief Process an incoming object array and rebuild the obstacle list.
   *
   * Filters out the ego vehicle, objects behind the ego, objects beyond the
   * lookahead, and objects beyond the lateral corridor. Long objects are
   * represented as three circles.
   *
   * @param msg   Incoming object array from /carla/objects.
   * @param ego_x Ego x in map frame.
   * @param ego_y Ego y in map frame.
   * @param ego_yaw Ego heading in radians.
   */
  void update(
    const derived_object_msgs::msg::ObjectArray & msg, double ego_x, double ego_y, double ego_yaw);

  /**
   * @brief Return the current filtered and inflated obstacle list.
   */
  const std::vector<Obstacle> & obstacles() const { return obstacles_; }

  /**
   * @brief Check whether any obstacle circle is ahead of the ego in its lane.
   *
   * The lateral gate is widened when avoidance is already active so the latch
   * is not dropped prematurely while the ego is beside a large obstacle.
   *
   * @param ego_pose Current ego pose.
   * @param avoidance_active True when AVOID_STATIC_OBSTACLE is the current behavior.
   * @return True if at least one obstacle is in the forward corridor.
   */
  bool obstacleAhead(const Pose & ego_pose, bool avoidance_active) const;

private:
  ObstacleProcessorParams params_;
  VehicleParams vehicle_;
  std::vector<Obstacle> obstacles_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__OBSTACLE_PROCESSOR_HPP_
