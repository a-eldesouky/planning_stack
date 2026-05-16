#ifndef LOCAL_PLANNER__PLANNING_TYPES_HPP_
#define LOCAL_PLANNER__PLANNING_TYPES_HPP_

#include <string>
#include <vector>

namespace local_planner
{

/**
 * @brief 2D/3D pose in the map frame: used as the common position type.
 */
struct Pose
{
  double x{0.0};        ///< x in map frame [m]
  double y{0.0};        ///< y in map frame [m]
  double z{0.0};        ///< z in map frame [m]: carried through, not used in 2D planning
  double heading{0.0};  ///< yaw in map frame [rad]
};

/**
 * @brief Single point in the local geometric trajectory output.
 */
struct TrajectoryPoint
{
  Pose pose;
  double curvature{0.0};         ///< Path curvature κ = 1/R [1/m]
  double max_velocity_mps{1e6};  ///< Per-point speed constraint [m/s]: upper bound for the velocity profiler
  bool is_stop_point{false};     ///< True if ego must reach zero at this point
  std::string stop_reason{};     ///< Human-readable stop label for controller logging
};

/**
 * @brief Inflated obstacle circle in the 2D map plane.
 */
struct Obstacle
{
  double x{0.0};       ///< Obstacle center x in map frame [m]
  double y{0.0};       ///< Obstacle center y in map frame [m]
  double radius{0.0};  ///< Inflated radius = half_diagonal + obstacle_inflation [m]
};

/**
 * @brief Centerline point used to constrain Hybrid A* to the local route corridor.
 */
struct CorridorPoint
{
  double x{0.0};
  double y{0.0};
  double half_width{3.0};  ///< Road boundary half-width at this point [m]
};

/**
 * @brief Route corridor used as a lightweight drivable-region constraint.
 */
struct RouteCorridor
{
  std::vector<CorridorPoint> centerline;
  double half_width{3.0};  ///< Global fallback half-width [m]
};

/**
 * @brief Kinematic parameters of the M2 Urban vehicle.
 */
struct VehicleParams
{
  double lf{0.7725};
  double lr{0.7725};
  double wheelbase{1.545};
  double width{1.45};
  double length{2.3};
  double max_steering_angle{0.2792};
  double max_steering_rate{0.17};
  double min_turning_radius{5.39};
};

/**
 * @brief Configuration parameters for the Hybrid A* search.
 */
struct HybridAStarParams
{
  double grid_resolution{0.5};
  int heading_bins{72};
  int steering_samples{7};
  double step_size{1.0};
  int max_iterations{15000};
  double obstacle_inflation{0.8};
  double obstacle_lookahead{30.0};
  double heuristic_weight{1.5};
  double goal_position_tolerance{1.5};
  double goal_heading_tolerance{0.35};
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__PLANNING_TYPES_HPP_
