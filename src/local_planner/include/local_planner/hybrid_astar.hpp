#ifndef LOCAL_PLANNER__HYBRID_ASTAR_HPP_
#define LOCAL_PLANNER__HYBRID_ASTAR_HPP_

#include <cmath>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "local_planner/planning_types.hpp"

namespace local_planner
{

/**
 * @brief Internal search node for Hybrid A*.
 *
 * Not part of planning_types.hpp because this struct is private to the
 * HybridAStar algorithm: no other module needs to know about it.
 */
struct HybridNode
{
  Pose pose{};
  double g{0.0};  ///< Cost from start = accumulated arc length [m]
  double f{0.0};  ///< Total cost estimate = g + h
  int grid_x{0};
  int grid_y{0};
  int grid_h{0};
  int parent_index{-1};        ///< Index into closed_list for backtracking (-1 = start)
  double steering_angle{0.0};  ///< Steering applied to reach this node [rad]

  bool operator>(const HybridNode & other) const { return f > other.f; }
};

/**
 * @brief Result returned by HybridAStar::plan().
 */
struct HybridAStarResult
{
  bool success{false};
  std::vector<TrajectoryPoint> path;  ///< Ordered path start->goal (empty on failure)
  std::string reason;                 ///< Human-readable failure reason for logging
};

/**
 * @class HybridAStar
 * @brief Pure C++ Hybrid A* planner for kinematically feasible obstacle avoidance.
 *
 * Owns only the search algorithm. No ROS dependency, no smoothing.
 * CubicSpline post-processing is handled by LocalPlannerNode after plan() returns.
 */
class HybridAStar
{
public:
  explicit HybridAStar(const VehicleParams & vehicle_params, const HybridAStarParams & params);

  /**
   * @brief Plan a kinematically feasible, collision-free path.
   *
   * @param start            Ego pose in map frame
   * @param goal             Lookahead endpoint pose in map frame
   * @param obstacles        Inflated obstacle circles within lookahead range
   * @param corridor         Optional route corridor constraint (nullptr = no corridor check)
   * @param max_velocity_mps Speed cap stamped on every output TrajectoryPoint [m/s]
   * @return HybridAStarResult: path is empty when success == false
   */
  [[nodiscard]] HybridAStarResult plan(
    const Pose & start, const Pose & goal, const std::vector<Obstacle> & obstacles,
    const RouteCorridor * corridor = nullptr, double max_velocity_mps = 1e6) const;

  /**
   * @brief Approximate vehicle footprint collision check.
   *
   * Samples 5 points from rear bumper to front bumper along the vehicle
   * centerline, each inflated by half vehicle width.
   */
  bool isCollisionFree(
    const Pose & pose, const std::vector<Obstacle> & obstacles) const;

  /**
   * @brief Validate every point of a trajectory against obstacles and corridor.
   *
   * Used to safety-check a spline-smoothed path before publishing it.
   * A cubic spline fitted through A* nodes may overshoot around tight turns
   * or narrow corridors, so the smoothed path must be rechecked point-by-point.
   *
   * @param path      Trajectory to validate (typically the smoothed A* output)
   * @param obstacles Inflated obstacle circles in the planning frame
   * @param corridor  Route corridor constraint (nullptr = skip corridor check)
   * @return True if every point is collision-free and inside the corridor
   */
  bool isTrajectorySafe(
    const std::vector<TrajectoryPoint> & path, const std::vector<Obstacle> & obstacles,
    const RouteCorridor * corridor) const;

private:
  static constexpr double kTwoPi = 2 * M_PI;  ///< Precompute for efficiency in angle normalization

  HybridNode applyMotion(const HybridNode & node, double steering_rad) const;
  Pose integrate(const Pose & start, double steering_rad, double distance) const;

  bool isMotionCollisionFree(
    const Pose & start, double steering_rad, const std::vector<Obstacle> & obstacles) const;

  bool isInsideCorridor(const Pose & pose, const RouteCorridor * corridor) const;

  bool isMotionInsideCorridor(
    const Pose & start, double steering_rad, const RouteCorridor * corridor) const;

  std::vector<double> generateSteeringAngles() const;

  double heuristic(const Pose & node_pose, const Pose & goal) const;
  double dubinsLength(const Pose & start, const Pose & goal, double radius) const;

  double dubinsLSL(double d, double alpha, double beta) const;
  double dubinsRSR(double d, double alpha, double beta) const;
  double dubinsLSR(double d, double alpha, double beta) const;
  double dubinsRSL(double d, double alpha, double beta) const;
  double dubinsRLR(double d, double alpha, double beta) const;
  double dubinsLRL(double d, double alpha, double beta) const;

  std::size_t toGridKey(const Pose & pose) const;
  int toGridX(double x) const;
  int toGridY(double y) const;
  int toGridH(double heading) const;

  bool isGoal(const HybridNode & node, const Pose & goal) const;

  std::vector<TrajectoryPoint> extractPath(
    const std::vector<HybridNode> & closed_list, int goal_idx, double max_velocity_mps) const;

  double normalizeAngle(double angle) const;
  double normalizeAngleSigned(double angle) const;
  double mod2pi(double angle) const;

  VehicleParams vehicle_;
  HybridAStarParams params_;
  double heading_bin_size_{0.0};
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__HYBRID_ASTAR_HPP_
