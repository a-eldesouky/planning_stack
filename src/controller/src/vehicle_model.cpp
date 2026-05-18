#include "controller/vehicle_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace controller
{

VehicleModel::VehicleModel(const VehicleModelParams & params)
: params_(params)
{
  if (params_.wheelbase <= 0.0) {
    throw std::invalid_argument("VehicleModel: wheelbase must be > 0");
  }
  if (params_.max_front_steer <= 0.0) {
    throw std::invalid_argument("VehicleModel: max_front_steer must be > 0");
  }
  if (params_.max_rear_steer < 0.0) {
    throw std::invalid_argument("VehicleModel: max_rear_steer must be >= 0");
  }
  if (params_.max_steering_rate <= 0.0) {
    throw std::invalid_argument("VehicleModel: max_steering_rate must be > 0");
  }
  if (params_.max_accel <= 0.0) {
    throw std::invalid_argument("VehicleModel: max_accel must be > 0");
  }
  if (params_.max_decel <= 0.0) {
    throw std::invalid_argument("VehicleModel: max_decel must be > 0");
  }
  if (params_.rear_steering_ratio < 0.0) {
    throw std::invalid_argument("VehicleModel: rear_steering_ratio must be >= 0");
  }
}

VehicleState VehicleModel::propagate(
  const VehicleState & state, const ControlInput & input, double dt) const
{
  const ControlInput u = clampInput(input);

  // CoM slip angle for double Ackermann
  const double beta = std::atan2(
    params_.lr * std::tan(u.delta_front) + params_.lf * std::tan(u.delta_rear), params_.wheelbase);

  // Heading rate
  const double psi_dot =
    (state.v / params_.wheelbase) * (std::tan(u.delta_front) - std::tan(u.delta_rear));

  VehicleState next;
  next.x = state.x + dt * state.v * std::cos(state.psi + beta);
  next.y = state.y + dt * state.v * std::sin(state.psi + beta);
  next.psi = normalizeAngle(state.psi + dt * psi_dot);
  next.v = std::max(0.0, state.v + dt * u.acceleration);

  return next;
}

ControlInput VehicleModel::coordinateSteering(double delta_front, double acceleration) const
{
  ControlInput input;
  input.delta_front = delta_front;
  input.acceleration = acceleration;

  if (params_.enable_rear_steering) {
    // Counter-phase rear steering reduces turning radius
    input.delta_rear = std::clamp(
      -params_.rear_steering_ratio * delta_front, -params_.max_rear_steer, params_.max_rear_steer);
  } else {
    input.delta_rear = 0.0;
  }

  return input;
}

ControlInput VehicleModel::clampInput(const ControlInput & input) const
{
  return ControlInput{
    std::clamp(input.delta_front, -params_.max_front_steer, params_.max_front_steer),
    std::clamp(input.delta_rear, -params_.max_rear_steer, params_.max_rear_steer),
    std::clamp(input.acceleration, -params_.max_decel, params_.max_accel)};
}

double VehicleModel::applySteeringRateLimit(double desired, double previous, double dt) const
{
  const double max_change = params_.max_steering_rate * dt;
  return std::clamp(
    previous + std::clamp(desired - previous, -max_change, max_change), -params_.max_front_steer,
    params_.max_front_steer);
}

CarlaControl VehicleModel::toCarlaControl(const ControlInput & input) const
{
  CarlaControl cmd;
  cmd.steer = std::clamp(input.delta_front / params_.max_front_steer, -1.0, 1.0);

  if (input.acceleration >= 0.0) {
    cmd.throttle = std::clamp(input.acceleration / params_.max_accel, 0.0, 1.0);
    cmd.brake = 0.0;
  } else {
    cmd.throttle = 0.0;
    cmd.brake = std::clamp(-input.acceleration / params_.max_decel, 0.0, 1.0);
  }

  return cmd;
}

double VehicleModel::computeCTE(
  const VehicleState & state, double path_x, double path_y, double path_psi)
{
  const double dx = state.x - path_x;
  const double dy = state.y - path_y;
  return -std::sin(path_psi) * dx + std::cos(path_psi) * dy;
}

double VehicleModel::computeHeadingError(const VehicleState & state, double path_psi)
{
  return normalizeAngle(state.psi - path_psi);
}

double VehicleModel::normalizeAngle(double angle)
{
  if (!std::isfinite(angle)) {
    return 0.0;
  }
  angle = std::remainder(angle, 2.0 * kPi);
  return angle;
}

}  // namespace controller
