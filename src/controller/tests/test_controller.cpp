#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "controller/pure_pursuit_controller.hpp"
#include "controller/vehicle_model.hpp"

namespace
{

using controller::ControlInput;
using controller::PurePursuitController;
using controller::PurePursuitControllerParams;
using controller::TrajectoryPoint;
using controller::VehicleModel;
using controller::VehicleModelParams;
using controller::VehicleState;

constexpr double kPi = 3.14159265358979323846;

// Matches config/controller_params.yaml (front steering only, as actuated in CARLA).
VehicleModelParams frontSteerModel()
{
  VehicleModelParams p;
  p.wheelbase = 1.545;
  p.lf = 0.7725;
  p.lr = 0.7725;
  p.max_front_steer = 0.2792;
  p.max_rear_steer = 0.2792;
  p.max_steering_rate = 0.17;
  p.max_accel = 3.0;
  p.max_decel = 4.0;
  p.enable_rear_steering = false;
  return p;
}

PurePursuitControllerParams yamlControllerParams()
{
  PurePursuitControllerParams p;
  p.lookahead_distance = 6.0;
  p.min_lookahead_distance = 4.0;
  p.lookahead_time_gain = 0.3;
  p.kp_speed = 0.8;
  p.ki_speed = 0.05;
  p.kd_speed = 0.02;
  p.integral_limit = 35.0;
  return p;
}

std::vector<TrajectoryPoint> straightLine(double y, double length, double spacing, double v)
{
  std::vector<TrajectoryPoint> pts;
  for (double x = 0.0; x <= length; x += spacing) {
    pts.push_back(TrajectoryPoint{x, y, 0.0, v, false, {}});
  }
  return pts;
}

// Counter-clockwise circle centered at (0, radius), starting at the origin heading +x.
std::vector<TrajectoryPoint> circle(double radius, double turns, double spacing, double v)
{
  std::vector<TrajectoryPoint> pts;
  const double total = turns * 2.0 * kPi * radius;
  for (double s = 0.0; s <= total; s += spacing) {
    const double th = s / radius;
    pts.push_back(
      TrajectoryPoint{radius * std::sin(th), radius * (1.0 - std::cos(th)), th, v, false, {}});
  }
  return pts;
}

// Points from the one nearest the ego onwards, like the ROS node feeds the controller.
// The search only moves forward from the previous nearest index (as the node's sliding
// window does), so a path that revisits the same place is followed in order.
std::vector<TrajectoryPoint> windowAhead(
  const std::vector<TrajectoryPoint> & path, const VehicleState & s, std::size_t count,
  std::size_t & nearest)
{
  constexpr std::size_t kSearchAhead = 40;
  const std::size_t search_end = std::min(path.size(), nearest + kSearchAhead);
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = nearest; i < search_end; ++i) {
    const double d = std::hypot(path[i].x - s.x, path[i].y - s.y);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }
  const std::size_t end = std::min(path.size(), nearest + count);
  return {path.begin() + static_cast<std::ptrdiff_t>(nearest),
    path.begin() + static_cast<std::ptrdiff_t>(end)};
}

struct SimResult
{
  double max_abs_error{0.0};
  double rms_error{0.0};
  VehicleState final_state;
};

// Closed loop with the kinematic model, including the steering-rate limit the node applies.
template<typename ErrorFn>
SimResult simulate(
  const std::vector<TrajectoryPoint> & path, VehicleState state, double duration,
  double settle_time, ErrorFn lateral_error)
{
  const VehicleModelParams mp = frontSteerModel();
  const VehicleModel model{mp};
  PurePursuitController pp{yamlControllerParams(), mp};

  constexpr double kDt = 0.05;
  double steer = 0.0;
  double sum_sq = 0.0;
  int samples = 0;
  std::size_t nearest = 0;
  SimResult r;

  for (double t = 0.0; t < duration; t += kDt) {
    const auto window = windowAhead(path, state, 80, nearest);
    const auto out = pp.computeCommand(state, window, kDt);
    EXPECT_TRUE(out.valid) << "invalid command at t=" << t << " s, window=" << window.size()
                           << " points, pose=(" << state.x << ", " << state.y << ")";
    steer = model.applySteeringRateLimit(out.delta_front, steer, kDt);
    state = model.propagate(state, model.coordinateSteering(steer, out.acceleration), kDt);

    if (t >= settle_time) {
      const double e = lateral_error(state);
      r.max_abs_error = std::max(r.max_abs_error, std::abs(e));
      sum_sq += e * e;
      ++samples;
    }
  }
  r.rms_error = samples > 0 ? std::sqrt(sum_sq / samples) : 0.0;
  r.final_state = state;
  return r;
}

}  // namespace

TEST(PurePursuit, RejectsShortTrajectoryOrBadTimestep)
{
  PurePursuitController pp{yamlControllerParams(), frontSteerModel()};
  const VehicleState s{};
  EXPECT_FALSE(pp.computeCommand(s, {}, 0.05).valid);
  EXPECT_FALSE(pp.computeCommand(s, straightLine(0.0, 0.5, 1.0, 5.0), 0.05).valid);
  EXPECT_FALSE(pp.computeCommand(s, straightLine(0.0, 20.0, 1.0, 5.0), 0.0).valid);
}

TEST(PurePursuit, StraightAheadGivesZeroSteer)
{
  PurePursuitController pp{yamlControllerParams(), frontSteerModel()};
  const auto out = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 5.0},
      straightLine(0.0, 30.0, 0.5, 5.0), 0.05);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.delta_front, 0.0, 1e-9);
  EXPECT_NEAR(out.cross_track_error, 0.0, 1e-9);
}

TEST(PurePursuit, SteersTowardThePath)
{
  PurePursuitController pp{yamlControllerParams(), frontSteerModel()};
  // ROS convention: +y is left, positive steer turns left.
  const auto left = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 5.0},
      straightLine(1.0, 30.0, 0.5, 5.0), 0.05);
  const auto right = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 5.0},
      straightLine(-1.0, 30.0, 0.5, 5.0), 0.05);
  EXPECT_GT(left.delta_front, 0.0);
  EXPECT_LT(right.delta_front, 0.0);
  EXPECT_NEAR(left.delta_front, -right.delta_front, 1e-9);
}

TEST(PurePursuit, CircleGivesAckermannSteerAngle)
{
  const auto mp = frontSteerModel();
  PurePursuitController pp{yamlControllerParams(), mp};
  constexpr double kRadius = 20.0;
  // Any lookahead point on the circle yields curvature 1/R for a vehicle on the circle.
  const auto out = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 5.0},
      circle(kRadius, 0.5, 0.25, 5.0), 0.05);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(out.delta_front, std::atan(mp.wheelbase / kRadius), 1e-3);
}

TEST(PurePursuit, SteeringSaturatesAtLimit)
{
  const auto mp = frontSteerModel();
  PurePursuitController pp{yamlControllerParams(), mp};
  const auto out = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 1.0},
      circle(1.0, 0.5, 0.05, 1.0), 0.05);
  EXPECT_NEAR(std::abs(out.delta_front), mp.max_front_steer, 1e-9);
}

TEST(PurePursuit, StopPointAtFrontCommandsBraking)
{
  PurePursuitController pp{yamlControllerParams(), frontSteerModel()};
  auto traj = straightLine(0.0, 20.0, 0.5, 5.0);
  traj.front().is_stop_point = true;
  traj.front().v = 0.0;
  const auto out = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 4.0}, traj, 0.05);
  EXPECT_DOUBLE_EQ(out.target_speed, 0.0);
  EXPECT_LT(out.acceleration, 0.0);
}

TEST(PurePursuit, AccelerationIsClampedToVehicleLimits)
{
  const auto mp = frontSteerModel();
  PurePursuitController pp{yamlControllerParams(), mp};
  const auto speed_up = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 0.0},
      straightLine(0.0, 30.0, 0.5, 30.0), 0.05);
  EXPECT_LE(speed_up.acceleration, mp.max_accel);
  pp.reset();
  const auto slow_down = pp.computeCommand(VehicleState{0.0, 0.0, 0.0, 30.0},
      straightLine(0.0, 30.0, 0.5, 0.0), 0.05);
  EXPECT_GE(slow_down.acceleration, -mp.max_decel);
}

TEST(VehicleModelTest, RejectsInvalidParams)
{
  auto p = frontSteerModel();
  p.wheelbase = 0.0;
  EXPECT_THROW(VehicleModel{p}, std::invalid_argument);
  p = frontSteerModel();
  p.max_decel = -1.0;
  EXPECT_THROW(VehicleModel{p}, std::invalid_argument);
}

TEST(VehicleModelTest, NormalizeAngleWrapsAndHandlesNonFinite)
{
  // 3*pi wraps to the boundary; either sign of pi is a correct representative.
  EXPECT_NEAR(std::abs(VehicleModel::normalizeAngle(3.0 * kPi)), kPi, 1e-9);
  EXPECT_NEAR(VehicleModel::normalizeAngle(-2.5 * kPi), -0.5 * kPi, 1e-9);
  EXPECT_NEAR(VehicleModel::normalizeAngle(1e9), std::remainder(1e9, 2.0 * kPi), 1e-6);
  EXPECT_DOUBLE_EQ(VehicleModel::normalizeAngle(std::numeric_limits<double>::infinity()), 0.0);
  EXPECT_DOUBLE_EQ(VehicleModel::normalizeAngle(std::nan("")), 0.0);
}

TEST(VehicleModelTest, SteeringRateLimitBoundsChange)
{
  const auto mp = frontSteerModel();
  const VehicleModel model{mp};
  const double next = model.applySteeringRateLimit(mp.max_front_steer, 0.0, 0.1);
  EXPECT_NEAR(next, mp.max_steering_rate * 0.1, 1e-12);
}

TEST(VehicleModelTest, CarlaCommandIsNormalized)
{
  const auto mp = frontSteerModel();
  const VehicleModel model{mp};
  const auto cmd = model.toCarlaControl(ControlInput{mp.max_front_steer * 2.0, 0.0, -100.0});
  EXPECT_DOUBLE_EQ(cmd.steer, 1.0);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 1.0);
}

TEST(ClosedLoop, ConvergesToStraightLineFromLateralOffset)
{
  const auto path = straightLine(0.0, 400.0, 0.5, 5.0);
  const auto r = simulate(path, VehicleState{0.0, 1.5, 0.0, 5.0}, 30.0, 15.0,
      [](const VehicleState & s) {return s.y;});
  std::cout << "[closed-loop straight] max |e| = " << r.max_abs_error
            << " m, rms = " << r.rms_error << " m\n";
  EXPECT_LT(r.max_abs_error, 0.10);
}

TEST(ClosedLoop, TracksCircleAtCruiseSpeed)
{
  constexpr double kRadius = 20.0;
  const auto path = circle(kRadius, 2.0, 0.5, 5.0);
  const auto r = simulate(path, VehicleState{0.0, 0.0, 0.0, 5.0}, 30.0, 10.0,
      [](const VehicleState & s) {return std::hypot(s.x, s.y - kRadius) - kRadius;});
  std::cout << "[closed-loop circle R=20 m] max |e| = " << r.max_abs_error
            << " m, rms = " << r.rms_error << " m\n";
  EXPECT_LT(r.max_abs_error, 0.50);
  EXPECT_NEAR(r.final_state.v, 5.0, 0.5);
}
