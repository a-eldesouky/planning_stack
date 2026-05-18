#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

#include "velocity_profiler/velocity_profiler.hpp"

namespace
{

using velocity_profiler::ProfilerParams;
using velocity_profiler::StampedWaypoint;
using velocity_profiler::VelocityProfiler;

constexpr double kTol = 1e-6;

// Straight path along +x with constant spacing.
std::vector<StampedWaypoint> straightPath(std::size_t n, double spacing)
{
  std::vector<StampedWaypoint> wps(n);
  for (std::size_t i = 0; i < n; ++i) {
    wps[i].x = static_cast<double>(i) * spacing;
    wps[i].y = 0.0;
    wps[i].yaw = 0.0;
  }
  return wps;
}

// Straight approach followed by a constant-radius left arc.
std::vector<StampedWaypoint> straightThenArc(
  std::size_t n_straight, std::size_t n_arc, double spacing, double radius)
{
  std::vector<StampedWaypoint> wps = straightPath(n_straight, spacing);
  const double x0 = wps.back().x;
  const double dtheta = spacing / radius;
  for (std::size_t k = 1; k <= n_arc; ++k) {
    const double theta = static_cast<double>(k) * dtheta;
    StampedWaypoint wp;
    wp.x = x0 + radius * std::sin(theta);
    wp.y = radius * (1.0 - std::cos(theta));
    wp.yaw = theta;
    wps.push_back(wp);
  }
  return wps;
}

// Longitudinal acceleration implied by consecutive speeds over the profiler's ds.
double impliedAccel(const StampedWaypoint & prev, const StampedWaypoint & curr)
{
  return (curr.velocity * curr.velocity - prev.velocity * prev.velocity) / (2.0 * curr.ds);
}

void expectWithinAccelLimits(const std::vector<StampedWaypoint> & out, const ProfilerParams & p)
{
  for (std::size_t i = 1; i < out.size(); ++i) {
    const double a = impliedAccel(out[i - 1], out[i]);
    EXPECT_LE(a, p.a_max + kTol) << "a_max exceeded between points " << i - 1 << " and " << i;
    EXPECT_GE(a, -p.a_dec - kTol) << "a_dec exceeded between points " << i - 1 << " and " << i;
  }
}

}  // namespace

TEST(VelocityProfiler, EmptyInputThrows)
{
  const VelocityProfiler profiler{ProfilerParams{}};
  EXPECT_THROW(profiler.generate({}, 0.0), std::invalid_argument);
}

TEST(VelocityProfiler, SinglePointIsZeroSpeed)
{
  const VelocityProfiler profiler{ProfilerParams{}};
  const auto out = profiler.generate(straightPath(1, 1.0), 5.0);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_DOUBLE_EQ(out[0].velocity, 0.0);
}

TEST(VelocityProfiler, InvalidParamsThrow)
{
  ProfilerParams p;
  p.min_ds = 0.0;
  EXPECT_THROW(VelocityProfiler{p}, std::invalid_argument);

  p = ProfilerParams{};
  p.cruise_speed = p.max_speed + 1.0;
  EXPECT_THROW(VelocityProfiler{p}, std::invalid_argument);

  p = ProfilerParams{};
  p.a_dec = -1.0;
  EXPECT_THROW(VelocityProfiler{p}, std::invalid_argument);
}

TEST(VelocityProfiler, AcceleratesFromRestToCruiseWithinLimits)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  const auto out = profiler.generate(straightPath(100, 1.0), 0.0);

  EXPECT_DOUBLE_EQ(out.front().velocity, 0.0);
  EXPECT_NEAR(out.back().velocity, p.cruise_speed, 1e-3);
  for (const auto & wp : out) {
    EXPECT_LE(wp.velocity, p.cruise_speed + kTol);
    EXPECT_GE(wp.velocity, 0.0);
  }
  expectWithinAccelLimits(out, p);
}

TEST(VelocityProfiler, StopsAtStopPointAndStaysStopped)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  auto wps = straightPath(60, 1.0);
  constexpr std::size_t kStop = 40;
  wps[kStop].is_stop_point = true;

  const auto out = profiler.generate(wps, p.cruise_speed);

  for (std::size_t i = kStop; i < out.size(); ++i) {
    EXPECT_DOUBLE_EQ(out[i].velocity, 0.0) << "nonzero speed at or past stop, index " << i;
  }
  expectWithinAccelLimits(out, p);
}

TEST(VelocityProfiler, RespectsPerPointSpeedCap)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  auto wps = straightPath(80, 1.0);
  for (std::size_t i = 50; i < 60; ++i) {
    wps[i].max_velocity_mps = 3.0;
  }

  const auto out = profiler.generate(wps, p.cruise_speed);

  for (std::size_t i = 50; i < 60; ++i) {
    EXPECT_LE(out[i].velocity, 3.0 + kTol) << "cap violated at index " << i;
  }
  // The slow-down into the capped zone must be a feasible ramp, not a step.
  expectWithinAccelLimits(out, p);
}

TEST(VelocityProfiler, CurvatureLimitsLateralAcceleration)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  constexpr double kRadius = 8.0;
  const auto out = profiler.generate(straightThenArc(40, 30, 1.0, kRadius), p.cruise_speed);

  // Well inside the arc the curvature estimate is exact, so lateral accel must hold.
  const double v_curve = std::sqrt(p.a_lat_max * kRadius);
  for (std::size_t i = 45; i < out.size(); ++i) {
    EXPECT_LE(out[i].velocity, v_curve + 1e-3) << "lateral limit violated at index " << i;
  }
  // Entering the curve from cruise speed must be a feasible braking ramp.
  expectWithinAccelLimits(out, p);
}

TEST(VelocityProfiler, HighEgoSpeedIntoShortStopStaysWithinBrakingLimit)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  auto wps = straightPath(30, 0.5);
  wps[20].is_stop_point = true;

  const auto out = profiler.generate(wps, p.cruise_speed);
  expectWithinAccelLimits(out, p);
}

TEST(VelocityProfiler, TimeFromStartIsMonotonic)
{
  const ProfilerParams p;
  const VelocityProfiler profiler{p};
  const auto out = profiler.generate(straightThenArc(40, 30, 1.0, 10.0), 2.0);

  for (std::size_t i = 1; i < out.size(); ++i) {
    EXPECT_GE(out[i].time_from_start, out[i - 1].time_from_start);
  }
}
