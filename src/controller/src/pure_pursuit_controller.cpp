#include "controller/pure_pursuit_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace controller
{

PurePursuitController::PurePursuitController(
  const PurePursuitControllerParams & controller_params, const VehicleModelParams & model_params)
: controller_params_(controller_params), model_params_(model_params)
{
  // Validate safety-critical geometry and controller tuning once during construction.
  // Invalid values here can produce unstable steering or invalid PID behavior at runtime.
  if (model_params_.wheelbase <= 0.0) {
    throw std::invalid_argument("PurePursuitController: wheelbase must be positive");
  }
  if (controller_params_.lookahead_distance <= 0.0) {
    throw std::invalid_argument("PurePursuitController: lookahead_distance must be positive");
  }
  if (controller_params_.min_lookahead_distance <= 0.0) {
    throw std::invalid_argument("PurePursuitController: min_lookahead_distance must be positive");
  }
  if (controller_params_.lookahead_time_gain < 0.0) {
    throw std::invalid_argument("PurePursuitController: lookahead_time_gain must be non-negative");
  }
}

ControllerOutput PurePursuitController::computeCommand(
  const VehicleState & state, const std::vector<TrajectoryPoint> & trajectory, double dt)
{
  // A valid Pure Pursuit command needs at least two trajectory points and a positive control period.
  // Returning the default output keeps valid=false and lets the caller publish a safe stop command.
  if (trajectory.size() < 2 || dt <= 0.0) {
    return {};
  }

  const double lookahead = dynamicLookahead(state.v);

  // Select the lookahead target by walking forward along the already-trimmed local trajectory.
  // This assumes trajectory[0] is close to the ego vehicle, which is handled by the ROS node.
  std::size_t target_index = 0;
  double accumulated_distance = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double dx = trajectory[i].x - trajectory[i - 1].x;
    const double dy = trajectory[i].y - trajectory[i - 1].y;
    accumulated_distance += std::hypot(dx, dy);
    target_index = i;

    if (accumulated_distance >= lookahead) {
      break;
    }
  }

  const auto & target = trajectory[target_index];
  const double dx = target.x - state.x;
  const double dy = target.y - state.y;

  // Transform the selected lookahead target from map frame into ego-vehicle frame.
  // local_x is forward distance, local_y is lateral offset from the vehicle centerline.
  const double local_x = std::cos(state.psi) * dx + std::sin(state.psi) * dy;
  const double local_y = -std::sin(state.psi) * dx + std::cos(state.psi) * dy;
  const double distance_sq = std::max(local_x * local_x + local_y * local_y, kEpsilon);

  // Pure Pursuit curvature in ego frame: kappa = 2 * y_l / Ld^2.
  // Positive local_y produces a steering command toward the left side of the vehicle frame.
  const double curvature = 2.0 * local_y / distance_sq;
  const double delta_front = frontSteerFromCurvature(curvature);

  // Use the velocity-profiled speed at the lookahead point.
  // v == 0.0 is a valid profiled speed (stop point); do not skip it.
  double target_speed = controller_params_.reference_speed;

  if (std::isfinite(target.v) && target.v >= 0.0) {
    // Primary source: velocity profiler output at the lookahead target.
    target_speed = target.v;
  } else {
    // Fallback for invalid data only (NaN, negative). Scan forward for the next
    // valid speed. v == 0.0 is accepted so a stop trajectory is not bypassed.
    for (std::size_t i = target_index; i < trajectory.size(); ++i) {
      if (std::isfinite(trajectory[i].v) && trajectory[i].v >= 0.0) {
        target_speed = trajectory[i].v;
        break;
      }
    }
  }

  // Hard stop guard: if the nearest trajectory point is an explicit stop with
  // near-zero speed, override any lookahead-derived target with zero.
  // Close-range enforcement (dist <= 1.5 m) is also handled upstream in the
  // ROS node before this function is called.
  if (trajectory.front().is_stop_point && trajectory.front().v <= 0.05) {
    target_speed = 0.0;
  }

  ControllerOutput output;
  output.valid = true;
  output.delta_front = delta_front;
  output.acceleration = computeSpeedAcceleration(target_speed, state.v, dt);
  output.target_speed = target_speed;
  output.cross_track_error = local_y;
  output.heading_error = VehicleModel::normalizeAngle(target.psi - state.psi);
  output.lookahead_distance = lookahead;
  return output;
}

void PurePursuitController::reset()
{
  // Clear stateful PID memory. This prevents stale integral/derivative terms from affecting the
  // next valid command after a stop, timeout, or controller interruption.
  speed_error_integral_ = 0.0;
  previous_speed_error_ = 0.0;
  has_previous_speed_error_ = false;
}

double PurePursuitController::computeSpeedAcceleration(
  double target_speed, double current_speed, double dt)
{
  const double error = target_speed - current_speed;

  // Clamp the integral term to prevent windup when the vehicle cannot reach the target speed quickly.
  speed_error_integral_ = std::clamp(
    speed_error_integral_ + error * dt, -controller_params_.integral_limit,
    controller_params_.integral_limit);

  // Skip derivative action on the first update because there is no previous error sample yet.
  const double derivative = has_previous_speed_error_ ? (error - previous_speed_error_) / dt : 0.0;
  previous_speed_error_ = error;
  has_previous_speed_error_ = true;

  const double acceleration = controller_params_.kp_speed * error +
                              controller_params_.ki_speed * speed_error_integral_ +
                              controller_params_.kd_speed * derivative;

  // Keep the acceleration request within the configured vehicle capability.
  return std::clamp(acceleration, -model_params_.max_decel, model_params_.max_accel);
}

double PurePursuitController::frontSteerFromCurvature(double curvature) const
{
  const double yaw_rate_term = curvature * model_params_.wheelbase;

  // Standard front-steering Ackermann approximation.
  // For a bicycle model: delta = atan(curvature * wheelbase).
  if (
    !model_params_.enable_rear_steering || std::abs(model_params_.rear_steering_ratio) < kEpsilon)
  {
    return std::clamp(
      std::atan(yaw_rate_term), -model_params_.max_front_steer, model_params_.max_front_steer);
  }

  // Double-Ackermann approximation.
  //
  // Solve:
  //   tan(delta_f) - tan(delta_r) = curvature * L
  //
  // With counter-phase rear steering:
  //   delta_r = -rear_steering_ratio * delta_f
  //
  // This becomes:
  //   tan(delta_f) + tan(ratio * delta_f) = |curvature * L|
  //
  // A bounded binary search is used because the relation is nonlinear and monotonic
  // within the configured steering limits.
  const double sign = yaw_rate_term >= 0.0 ? 1.0 : -1.0;
  const double target = std::abs(yaw_rate_term);

  double low = 0.0;
  double high = model_params_.max_front_steer;

  for (int i = 0; i < 32; ++i) {
    const double mid = 0.5 * (low + high);

    // Rear steering magnitude is limited independently from front steering.
    const double rear =
      std::clamp(model_params_.rear_steering_ratio * mid, 0.0, model_params_.max_rear_steer);

    const double achieved = std::tan(mid) + std::tan(rear);
    if (achieved < target) {
      low = mid;
    } else {
      high = mid;
    }
  }

  return sign * high;
}

double PurePursuitController::dynamicLookahead(double speed) const
{
  // Speed-scaled lookahead improves stability at higher speed while preserving a minimum
  // lookahead distance for low-speed maneuvering.
  return std::max(
    controller_params_.min_lookahead_distance,
    controller_params_.lookahead_distance + controller_params_.lookahead_time_gain * speed);
}

}  // namespace controller
