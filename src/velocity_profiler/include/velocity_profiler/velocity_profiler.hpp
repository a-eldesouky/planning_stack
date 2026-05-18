#ifndef VELOCITY_PROFILER__VELOCITY_PROFILER_HPP_
#define VELOCITY_PROFILER__VELOCITY_PROFILER_HPP_

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace velocity_profiler
{
/**
 * @brief Internal working waypoint.
 *
 * The x/y/yaw and the local-planner constraint fields (max_velocity_mps, is_stop_point,
 * stop_reason) are copied verbatim from the incoming trajectory.
 * This stage fills velocity, acceleration, and time_from_start.
 */
struct StampedWaypoint
{
  double x{0.0};    ///< [m] in a local Cartesian frame (e.g. ENU)
  double y{0.0};    ///< [m] in a local Cartesian frame (e.g. ENU)
  double yaw{0.0};  ///< [rad] heading angle in the local frame

  double max_velocity_mps{1e6};  ///< Per-point speed cap from the local planner [m/s]
  bool is_stop_point{false};     ///< True → backward pass stops here
  std::string stop_reason{};     ///< Preserved into output for the controller

  double curvature{0.0};  ///< Estimated κ = dYaw / ds [1/m]
  double ds{0.0};         ///< Arc length step from previous waypoint [m]

  double velocity{0.0};         ///< Final target speed [m/s]
  double acceleration{0.0};     ///< Longitudinal acceleration [m/s²]
  double time_from_start{0.0};  ///< Elapsed time from first point [s]
};

/**
 * @brief Tuning parameters: loaded from YAML, no behavior state.
 */
struct ProfilerParams
{
  double cruise_speed{8.33};  ///< [m/s] = 30 km/h nominal target
  double max_speed{13.89};    ///< [m/s] = 50 km/h absolute cap

  double a_max{2.0};  ///< [m/s²] max longitudinal acceleration
  double a_dec{3.0};  ///< [m/s²] max deceleration
  double j_max{1.5};  ///< [m/s³] max jerk (heuristic smoothing)

  double a_lat_max{2.5};                 ///< [m/s²] max lateral acceleration for curvature cap
  double min_curvature_threshold{1e-4};  ///< [1/m] ignore curvature below this

  double min_ds{0.01};  ///< [m] minimum arc step (numerical stability)
};

/**
 * @brief Pure C++ velocity profiler.
 *
 * Input:  vector<StampedWaypoint> with geometry + local-planner constraints filled.
 * Output: same vector with velocity, acceleration, time_from_start filled.
 *
 * @note The only runtime context needed from outside is ego_speed (v[0]).
 *       All other constraints come from the waypoints themselves.
 */
class VelocityProfiler
{
public:
  /**
 * @brief Construct a new Velocity Profiler object
 *
 * @param params
 */
  explicit VelocityProfiler(const ProfilerParams & params);

  void setParams(const ProfilerParams & params);
  const ProfilerParams & params() const { return params_; }

  /**
   * @brief Generate velocity profile.
   *
   * @param wps       Waypoints from the local planner (max_velocity_mps + is_stop_point filled).
   * @param ego_speed Current ego speed [m/s]: used as v[0] in the forward pass.
   * @return          Same waypoints with velocity/acceleration/time_from_start filled.
   *
   * A single waypoint is returned with zero speed.
   *
   * @throws std::invalid_argument if wps is empty
   */
  std::vector<StampedWaypoint> generate(
    std::vector<StampedWaypoint> wps, double ego_speed) const;

private:
  void computeGeometry(std::vector<StampedWaypoint> & wps) const;

  /**
   * @brief Compute per-point speed cap.
   *
   * cap[i] = min(params.max_speed,
   *              point.max_velocity_mps,     ← from the local planner
   *              sqrt(a_lat_max / |kappa|))  ← curvature limit (added here)
   *
   * is_stop_point → cap[i] = 0.0, index recorded in stop_indices.
   */
  std::vector<double> computeSpeedCaps(
    const std::vector<StampedWaypoint> & wps, std::vector<int> & stop_indices) const;

  /**
   * @brief Perform forward pass to compute initial velocity profile.
   *
   * @param wps
   * @param caps
   * @param v0
   */
  void forwardPass(
    std::vector<StampedWaypoint> & wps, const std::vector<double> & caps, double v0) const;

  /**
   * @brief Perform backward pass to adjust velocities.
   *
   * @param wps
   * @param stop_indices
   */
  void backwardPass(
    std::vector<StampedWaypoint> & wps, const std::vector<int> & stop_indices) const;

  /// Lower speeds so that v[i]^2 <= v[i-1]^2 + 2 * a_max * ds[i] for every i.
  void limitAcceleration(std::vector<StampedWaypoint> & wps) const;

  /// Lower speeds so that v[i]^2 <= v[i+1]^2 + 2 * a_dec * ds[i+1] for every i.
  void limitDeceleration(std::vector<StampedWaypoint> & wps) const;

  /**
   * @brief Best-effort jerk smoothing: lowers speeds where the discrete jerk exceeds j_max.
   *
   * Acceleration and braking limits are re-applied afterwards, so j_max is not a hard bound.
   *
   * @param wps
   */
  void jerkSmoothing(std::vector<StampedWaypoint> & wps) const;

  /**
   * @brief Fill time_from_start based on velocity and ds.
   *
   * @param wps
   */
  void assignTimings(std::vector<StampedWaypoint> & wps) const;

  /**
   * @brief Validate params_ for sanity (e.g. non-negative, max_speed > cruise_speed, etc.).
   *
   */
  void validateParams() const;

  ProfilerParams params_;
};

}  // namespace velocity_profiler

#endif  // VELOCITY_PROFILER__VELOCITY_PROFILER_HPP_
