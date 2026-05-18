#ifndef CONTROLLER__VEHICLE_MODEL_HPP_
#define CONTROLLER__VEHICLE_MODEL_HPP_

#include <cmath>
#include <stdexcept>
#include <vector>

namespace controller
{

/**
 * @brief Ego vehicle state used by the controller and kinematic model.
 *
 * The state is expressed in the planning/map frame, using the vehicle center of
 * mass or controller reference point as the pose reference.
 */
struct VehicleState
{
  double x{0.0};    ///< Vehicle x position in map/planning frame [m].
  double y{0.0};    ///< Vehicle y position in map/planning frame [m].
  double psi{0.0};  ///< Vehicle heading/yaw angle normalized to [-pi, pi] [rad].
  double v{0.0};    ///< Longitudinal vehicle speed [m/s].
};

/**
 * @brief Physical control input before conversion to CARLA actuator commands.
 */
struct ControlInput
{
  double delta_front{0.0};  ///< Front steering angle command [rad].
  double delta_rear{0.0};   ///< Rear steering angle command [rad].
  double acceleration{
    0.0};  ///< Signed longitudinal acceleration; positive throttle, negative brake [m/s^2].
};

/**
 * @brief Normalized CARLA vehicle command representation.
 *
 * This structure keeps CARLA-specific actuator ranges separated from the pure
 * controller and vehicle-model calculations.
 */
struct CarlaControl
{
  double throttle{0.0};  ///< Normalized throttle command in range [0, 1].
  double brake{0.0};     ///< Normalized brake command in range [0, 1].
  double steer{0.0};     ///< Normalized steering command in range [-1, 1].
};

/**
 * @brief Vehicle geometry, actuator limits, and steering coordination parameters.
 *
 * These parameters are normally loaded from the controller YAML file and should
 * match the CARLA ego vehicle used during simulation.
 */
struct VehicleModelParams
{
  double wheelbase{1.545};  ///< Distance between front and rear axle centers [m].
  double lf{0.7725};        ///< Distance from center of mass/reference point to front axle [m].
  double lr{0.7725};        ///< Distance from center of mass/reference point to rear axle [m].
  double max_front_steer{0.2792};   ///< Maximum absolute front steering angle [rad].
  double max_rear_steer{0.1396};    ///< Maximum absolute rear steering angle [rad].
  double max_steering_rate{0.17};   ///< Maximum front steering rate [rad/s].
  double max_accel{2.0};            ///< Maximum positive acceleration command [m/s^2].
  double max_decel{4.0};            ///< Maximum braking/deceleration magnitude [m/s^2].
  double max_throttle{1.0};         ///< Maximum normalized throttle command.
  double max_brake{1.0};            ///< Maximum normalized brake command.
  bool enable_rear_steering{false};  ///< Model-only counter-phase rear steering (not actuated in CARLA).
  double rear_steering_ratio{0.3};  ///< Rear-to-front steering ratio for counter-phase steering.
};

/**
 * @class VehicleModel
 * @brief Kinematic bicycle model (optional model-only rear steering) and CARLA command conversion.
 *
 * Kinematic equations using a center-of-mass reference:
 *
 *   beta = atan2(lr * tan(delta_front) + lf * tan(delta_rear), wheelbase)
 *   x_dot = v * cos(psi + beta)
 *   y_dot = v * sin(psi + beta)
 *   psi_dot = (v / wheelbase) * (tan(delta_front) - tan(delta_rear))
 *   v_dot = acceleration
 */
class VehicleModel
{
public:
  /**
   * @brief Construct a vehicle model from geometry and actuator parameters.
   *
   * @param params Vehicle dimensions, limits, and steering coordination settings.
   *
   * @throws std::invalid_argument if critical vehicle dimensions or limits are invalid.
   */
  explicit VehicleModel(const VehicleModelParams & params);

  /**
   * @brief Propagate the vehicle state forward by one time step.
   *
   * Uses a kinematic double-Ackermann model with simple Euler integration.
   * This is mainly useful for prediction, validation, and controller testing.
   *
   * @param state Current vehicle state.
   * @param input Physical control input.
   * @param dt Integration time step [s].
   * @return Predicted vehicle state after dt.
   */
  VehicleState propagate(
    const VehicleState & state, const ControlInput & input, double dt) const;

  /**
   * @brief Compute coordinated front/rear steering commands.
   *
   * When rear steering is enabled, the rear steering command is generated from
   * the front steering command using the configured ratio. Otherwise rear
   * steering is set to zero.
   *
   * @param delta_front Desired front steering angle [rad].
   * @param acceleration Desired longitudinal acceleration [m/s^2].
   * @return Coordinated and unclamped control input.
   */
  ControlInput coordinateSteering(double delta_front, double acceleration) const;

  /**
   * @brief Clamp a physical control input to configured vehicle limits.
   *
   * @param input Raw control input.
   * @return Control input limited by steering and acceleration constraints.
   */
  ControlInput clampInput(const ControlInput & input) const;

  /**
   * @brief Apply steering-rate limiting to the front steering command.
   *
   * @param desired Desired front steering angle [rad].
   * @param previous Previous front steering command [rad].
   * @param dt Control time step [s].
   * @return Rate-limited front steering command [rad].
   */
  double applySteeringRateLimit(double desired, double previous, double dt) const;

  /**
   * @brief Convert physical control input to normalized actuator commands.
   *
   * Positive acceleration is mapped to throttle, while negative acceleration is
   * mapped to brake. Steering is normalized using the maximum front steering
   * angle.
   *
   * @param input Physical control input.
   * @return Normalized throttle, brake, and steering command.
   */
  CarlaControl toCarlaControl(const ControlInput & input) const;

  /**
   * @brief Compute signed cross-track error relative to a path pose.
   *
   * Positive error means the ego vehicle is to the left of the path heading.
   *
   * @param state Current ego vehicle state.
   * @param path_x Reference path x position [m].
   * @param path_y Reference path y position [m].
   * @param path_psi Reference path heading [rad].
   * @return Signed lateral error [m].
   */
  static double computeCTE(
    const VehicleState & state, double path_x, double path_y, double path_psi);

  /**
   * @brief Compute signed heading error relative to a path heading.
   *
   * @param state Current ego vehicle state.
   * @param path_psi Reference path heading [rad].
   * @return Normalized heading error in range [-pi, pi] [rad].
   */
  static double computeHeadingError(const VehicleState & state, double path_psi);

  /**
   * @brief Normalize an angle to the range [-pi, pi].
   *
   * @param angle Input angle [rad].
   * @return Normalized angle [rad].
   */
  static double normalizeAngle(double angle);

  /**
   * @brief Access the immutable vehicle model parameters.
   *
   * @return Vehicle model parameters used by this instance.
   */
  const VehicleModelParams & params() const { return params_; }

private:
  VehicleModelParams params_;  ///< Vehicle geometry, actuator limits, and steering settings.

  static constexpr double kPi =
    3.14159265358979323846;  ///< Pi constant used for angle normalization.
};

}  // namespace controller

#endif  // CONTROLLER__VEHICLE_MODEL_HPP_
