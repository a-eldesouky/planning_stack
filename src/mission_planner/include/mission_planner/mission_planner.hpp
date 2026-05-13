#ifndef MISSION_PLANNER__MISSION_PLANNER_HPP_
#define MISSION_PLANNER__MISSION_PLANNER_HPP_

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "hd_map/hd_map.hpp"

namespace mission_planner
{

/**
 * @brief 2D point used for centerline midpoint calculations.
 */
struct Point2D
{
  double x{0.0};  ///< X coordinate in map frame (metres)
  double y{0.0};  ///< Y coordinate in map frame (metres)
};

/**
 * @brief A single waypoint in the planned route.
 *
 * Coordinates are in the map frame.
 * heading_rad is the forward direction of travel at this point.
 */
struct RoutePoint
{
  double x{0.0};            ///< X coordinate in map frame (metres)
  double y{0.0};            ///< Y coordinate in map frame (metres)
  double z{0.0};            ///< Z coordinate in map frame (metres)
  double heading_rad{0.0};  ///< Forward heading in radians [0, 2π)
};

/**
 * @brief One lanelet segment in the planned route.
 *
 * Carries both the ordered waypoints and metadata that downstream
 * modules (Behavior Planner, Local Planner) need without re-querying
 * the HD map.
 */
struct RouteLanelet
{
  std::string id;                     ///< Lanelet ID from HD map
  bool is_junction{false};            ///< True if inside a junction
  std::vector<RoutePoint> waypoints;  ///< Ordered centerline waypoints
};

/**
 * @brief Full planned route from start pose to goal pose.
 *
 * Contains an ordered list of lanelets. Returned as
 * std::optional<Route> from plan(): nullopt means no path found.
 */
struct Route
{
  std::vector<RouteLanelet> lanelets;  ///< Ordered lanelets start → goal
  double total_length_m{0.0};          ///< Sum of all lanelet lengths

  /// @brief Returns true if the route contains no lanelets.
  bool empty() const { return lanelets.empty(); }
};

/**
 * @class MissionPlanner
 * @brief Global route planner using A* search on the HD map lanelet graph.
 *
 * Loads the HD map from a JSON file at construction, then plans routes
 * on demand via plan(). The planner is stateless after construction:
 * plan() is fully const and thread-safe for concurrent calls.
 *
 * ## Algorithm
 * A* search on the lanelet graph:
 * - Nodes  = lanelets
 * - Edges  = successor relationships from HD map
 * - Cost   = lanelet length_m
 * - H(n)   = Euclidean distance from lanelet centerline midpoint to goal
 *
 * ## Lane Snapping
 * Both start and goal use heading-aware snapping via findNearestLanelet().
 * This prevents snapping to opposite-direction or crossing lanelets at
 * junctions.
 */
class MissionPlanner
{
public:
  /**
     * @brief Construct and load the HD map.
     *
     * @param hd_map_path Absolute path to hd_map.json from the HD map generator.
     * @throws std::runtime_error if the map file cannot be loaded.
     */
  explicit MissionPlanner(const std::string & hd_map_path);

  /**
     * @brief Returns true if the HD map loaded successfully.
     */
  bool isReady() const { return map_ && map_->isLoaded(); }

  /**
   * @brief Expose the HD map for road geometry queries in routeToMsg.
   *
   * Used by MissionPlannerNode::routeToMsg() to fill road_half_width,
   * lane_half_width, and curvature per waypoint.
   *
   * @return Reference to the loaded HDMap.
   */
  const hd_map::HDMap & getMap() const { return *map_; }

  /**
     * @brief Plan a global route from current pose to goal position.
     *
     * Both start and goal are snapped to the nearest heading-compatible
     * lanelet before running A*. The RViz2 "2D Goal Pose" tool provides
     * heading for the goal via the drag direction.
     *
     * @param start  Current pose of the ego vehicle (from odometry)
     * @param goal   Desired goal pose (from RViz2)
     * @return                   Planned route, or std::nullopt if unreachable
     */
  [[nodiscard]] std::optional<Route> plan(RoutePoint start, RoutePoint goal) const;

  /**
     * @brief Find the nearest heading-compatible lanelet to a pose.
     *
     * Searches within kSnapRadiusM for centerline points whose heading
     * matches heading_rad within kHeadingThreshold. Falls back to raw
     * nearest lanelet if no heading-compatible candidate is found.
     *
     * @param pose Pose to snap (x, y, heading_rad)
     * @return Pointer to best matching lanelet, or nullptr if map is empty.
     */
  const hd_map::Lanelet * findNearestLanelet(const RoutePoint & pose) const;

private:
  /**
     * @brief Internal node representation for the A* priority queue.
     */
  struct AStarNode
  {
    std::string id;  ///< Lanelet ID
    double g{0.0};   ///< Accumulated cost from start
    double f{0.0};   ///< g + heuristic estimate to goal

    /// Min-heap ordering: lowest f expanded first.
    bool operator>(const AStarNode & other) const { return f > other.f; }
  };

  /**
     * @brief Run A* search between two lanelets.
     *
     * @param start_id  Starting lanelet ID
     * @param goal_id   Goal lanelet ID
     * @return          Ordered list of lanelet IDs start → goal,
     *                  empty if no path exists
     */
  std::vector<std::string> runAStar(
    const std::string & start_id, const std::string & goal_id) const;

  /**
   * @brief Straight-line distance from the end of a lanelet to the end of the goal lanelet.
   *
   * With edgeCost() as the step cost, the remaining driven distance from the end
   * of `lane` to the end of the goal is at least this straight line, so the
   * heuristic is admissible and consistent (up to millimetre rounding in the
   * map's length_m values). A* therefore returns the same route cost as Dijkstra;
   * tests/test_mission_planner.cpp checks this on many start/goal pairs.
   *
   * @param lane      Lanelet to evaluate
   * @param goal_end  Last centerline point of the goal lanelet (map frame)
   * @return          Lower bound on the remaining distance [m]
   */
  static double heuristic(const hd_map::Lanelet & lane, Point2D goal_end);

  /**
   * @brief Cost of moving from `from` into its successor `to`.
   *
   * Connection gap between the centerlines plus the full length of `to` [m].
   */
  static double edgeCost(const hd_map::Lanelet & from, const hd_map::Lanelet & to);

  /// Last centerline point of a lanelet in the map frame.
  static Point2D centerlineEnd(const hd_map::Lanelet & lane);

  /**
     * @brief Build a rich Route struct from an ordered lanelet ID list.
     *
     * Copies centerline waypoints and metadata into each RouteLanelet.
     * The first lanelet is trimmed to start from the centerline point
     * closest to (start_x, start_y), and the last lanelet is trimmed
     * to end at the closest point to (goal_x, goal_y). Middle lanelets
     * are included in full.
     *
     * This ensures the published path begins at the ego pose and ends
     * at the goal: not at lanelet entry/exit boundaries.
     *
     * @param lanelet_ids  Ordered list from runAStar()
     * @param start         Start pose for trimming (x, y)
     * @param goal          Goal pose for trimming (x, y)
     * @return             Fully populated and trimmed Route
     */
  Route buildRoute(const std::vector<std::string> & lanelet_ids, Point2D start, Point2D goal) const;

  std::unique_ptr<hd_map::HDMap> map_;  ///< Owns the loaded HD map

  static constexpr double kSnapRadiusM = 2.0;              ///< Snap search radius (metres)
  static constexpr double kHeadingThreshold = M_PI / 2.0;  ///< Max heading diff (90°)
};

}  // namespace mission_planner

#endif  // MISSION_PLANNER__MISSION_PLANNER_HPP_
