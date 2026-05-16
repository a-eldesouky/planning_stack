#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "local_planner/cubic_spline.hpp"
#include "local_planner/hybrid_astar.hpp"
#include "local_planner/planning_types.hpp"

namespace
{

using local_planner::CorridorPoint;
using local_planner::CubicSpline;
using local_planner::HybridAStar;
using local_planner::HybridAStarParams;
using local_planner::Obstacle;
using local_planner::Pose;
using local_planner::RouteCorridor;
using local_planner::TrajectoryPoint;
using local_planner::VehicleParams;

constexpr double kPi = 3.14159265358979323846;

double wrap(double a)
{
  return std::remainder(a, 2.0 * kPi);
}

RouteCorridor straightCorridor(double length, double half_width)
{
  RouteCorridor c;
  c.half_width = half_width;
  for (double x = -5.0; x <= length + 5.0; x += 0.5) {
    c.centerline.push_back(CorridorPoint{x, 0.0, half_width});
  }
  return c;
}

TrajectoryPoint point(double x, double y)
{
  TrajectoryPoint p;
  p.pose.x = x;
  p.pose.y = y;
  return p;
}

// Every step of the search integrates a bicycle arc, so the heading change per
// step can never exceed step_size / R_min.
void expectKinematicallyFeasible(
  const std::vector<TrajectoryPoint> & path, const VehicleParams & v, const HybridAStarParams & p)
{
  const double max_curvature = std::tan(v.max_steering_angle) / v.wheelbase;
  for (std::size_t i = 0; i < path.size(); ++i) {
    EXPECT_LE(std::abs(path[i].curvature), max_curvature + 1e-9) << "point " << i;
    if (i > 0) {
      const double dh = std::abs(wrap(path[i].pose.heading - path[i - 1].pose.heading));
      EXPECT_LE(dh, p.step_size * max_curvature + 1e-6) << "heading jump at point " << i;
    }
  }
}

}  // namespace

TEST(HybridAStar, PlansStraightPathInOpenSpace)
{
  const VehicleParams v;
  const HybridAStarParams p;
  const HybridAStar planner{v, p};

  const auto r = planner.plan(Pose{0.0, 0.0, 0.0, 0.0}, Pose{20.0, 0.0, 0.0, 0.0}, {});
  ASSERT_TRUE(r.success) << r.reason;
  ASSERT_GE(r.path.size(), 2u);
  EXPECT_DOUBLE_EQ(r.path.front().pose.x, 0.0);
  EXPECT_LT(std::hypot(r.path.back().pose.x - 20.0, r.path.back().pose.y), p.goal_position_tolerance);
  expectKinematicallyFeasible(r.path, v, p);
}

TEST(HybridAStar, AvoidsObstacleInsideCorridor)
{
  const VehicleParams v;
  const HybridAStarParams p;
  const HybridAStar planner{v, p};
  const RouteCorridor corridor = straightCorridor(30.0, 4.0);
  // A parked car half-blocking the lane: the path must swerve around it.
  const std::vector<Obstacle> obstacles{{12.0, -0.5, 1.6}};

  const auto r = planner.plan(
    Pose{0.0, 0.0, 0.0, 0.0}, Pose{25.0, 0.0, 0.0, 0.0}, obstacles, &corridor, 5.0);
  ASSERT_TRUE(r.success) << r.reason;

  EXPECT_TRUE(planner.isTrajectorySafe(r.path, obstacles, &corridor));
  double max_lateral = 0.0;
  for (const auto & pt : r.path) {
    EXPECT_TRUE(planner.isCollisionFree(pt.pose, obstacles));
    EXPECT_DOUBLE_EQ(pt.max_velocity_mps, 5.0);
    max_lateral = std::max(max_lateral, std::abs(pt.pose.y));
  }
  EXPECT_GT(max_lateral, 1.0) << "path did not leave the lane center to pass the obstacle";
  expectKinematicallyFeasible(r.path, v, p);
}

TEST(HybridAStar, FailsWhenCorridorIsFullyBlocked)
{
  const VehicleParams v;
  HybridAStarParams p;
  p.max_iterations = 3000;
  const HybridAStar planner{v, p};
  const RouteCorridor corridor = straightCorridor(30.0, 2.5);
  const std::vector<Obstacle> wall{{12.0, 0.0, 3.0}};

  const auto r = planner.plan(
    Pose{0.0, 0.0, 0.0, 0.0}, Pose{25.0, 0.0, 0.0, 0.0}, wall, &corridor);
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
  EXPECT_FALSE(r.reason.empty());
}

TEST(HybridAStar, RejectsGoalInsideObstacle)
{
  const HybridAStar planner{VehicleParams{}, HybridAStarParams{}};
  const auto r = planner.plan(
    Pose{0.0, 0.0, 0.0, 0.0}, Pose{20.0, 0.0, 0.0, 0.0}, {{20.0, 0.0, 1.0}});
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.path.empty());
}

TEST(CubicSpline, NeedsAtLeastTwoPoints)
{
  EXPECT_FALSE(CubicSpline::fit({}).valid());
  EXPECT_FALSE(CubicSpline::fit({point(1.0, 1.0)}).valid());
  EXPECT_TRUE(CubicSpline::fit({point(0.0, 0.0), point(1.0, 0.0)}).valid());
}

TEST(CubicSpline, StraightLineStaysStraight)
{
  std::vector<TrajectoryPoint> ctrl;
  for (int i = 0; i <= 10; ++i) {
    ctrl.push_back(point(2.0 * i, 0.0));
  }
  const auto spline = CubicSpline::fit(ctrl);
  ASSERT_TRUE(spline.valid());
  EXPECT_NEAR(spline.length(), 20.0, 1e-3);

  const auto out = spline.resample(0.5);
  ASSERT_GE(out.size(), 2u);
  for (const auto & pt : out) {
    EXPECT_NEAR(pt.pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pt.pose.heading, 0.0, 1e-9);
    EXPECT_NEAR(pt.curvature, 0.0, 1e-9);
  }
}

TEST(CubicSpline, KeepsEndpointsAndSpacing)
{
  std::vector<TrajectoryPoint> ctrl{point(0.0, 0.0), point(5.0, 1.0), point(10.0, 0.0),
    point(15.0, -1.0)};
  ctrl.back().is_stop_point = true;
  const auto out = CubicSpline::fit(ctrl).resample(0.5);
  ASSERT_GE(out.size(), 3u);

  EXPECT_DOUBLE_EQ(out.front().pose.x, 0.0);
  EXPECT_DOUBLE_EQ(out.front().pose.y, 0.0);
  EXPECT_DOUBLE_EQ(out.back().pose.x, 15.0);
  EXPECT_DOUBLE_EQ(out.back().pose.y, -1.0);
  EXPECT_TRUE(out.back().is_stop_point);

  // Interior samples are evenly spaced; the last step may be shorter.
  for (std::size_t i = 1; i + 1 < out.size(); ++i) {
    const double d = std::hypot(out[i].pose.x - out[i - 1].pose.x, out[i].pose.y - out[i - 1].pose.y);
    EXPECT_NEAR(d, 0.5, 0.05) << "spacing at sample " << i;
  }
}

TEST(CubicSpline, RecoversCircleCurvature)
{
  constexpr double kRadius = 10.0;
  std::vector<TrajectoryPoint> ctrl;
  for (int i = 0; i <= 16; ++i) {
    const double th = 0.1 * i;
    ctrl.push_back(point(kRadius * std::sin(th), kRadius * (1.0 - std::cos(th))));
  }
  const auto out = CubicSpline::fit(ctrl).resample(0.5);
  ASSERT_GT(out.size(), 10u);

  // Natural end conditions force zero curvature at the ends, so check the middle third.
  for (std::size_t i = out.size() / 3; i < 2 * out.size() / 3; ++i) {
    EXPECT_NEAR(out[i].curvature, 1.0 / kRadius, 0.01) << "sample " << i;
  }
}
