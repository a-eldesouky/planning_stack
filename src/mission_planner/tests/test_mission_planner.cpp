#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "mission_planner/mission_planner.hpp"

// Town01 HD map: test data
//
// File: share/hd_map/maps/Town01_hd_map.json, installed by the hd_map package
// Stats: 124 lanelets, 12 T-junctions
// Frame: ROS map frame (x forward, y left, heading in [-pi, pi]).
//
// Key lanelets used in tests:
//
//   lanelet_25_-1   road=25  heading=-1.572 rad   len=35m
//                   center ~ (154.0, -29.0)
//                   successors: lanelet_333_-1, lanelet_339_-1
//
//   lanelet_25_1    road=25  heading=1.571 rad
//                   opposite direction to lanelet_25_-1
//
//   lanelet_333_-1  road=333  is_junction=true
//                   junction lanelet connecting road 25 to road 9
//
//   lanelet_9_1     road=9   heading=-3.141 rad
//                   center ~ (123.0, -55.5)
//
//   lanelet_8_-1    road=8   heading=1.571 rad
//                   long road segment

#ifndef TOWN01_MAP_PATH
#define TOWN01_MAP_PATH "tests/data/Town01_hd_map.json"
#endif

// Fixture
class Town01PlannerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    planner_ = std::make_unique<mission_planner::MissionPlanner>(TOWN01_MAP_PATH);
    ASSERT_TRUE(planner_->isReady()) << "Failed to load: " << TOWN01_MAP_PATH;
  }

  std::unique_ptr<mission_planner::MissionPlanner> planner_;
};

// Test 1: Town01 map loads with expected counts
TEST_F(Town01PlannerTest, MapLoadsWithCorrectCounts)
{
  ASSERT_TRUE(planner_->isReady());

  EXPECT_EQ(planner_->getMap().getAllLanelets().size(), 124u);
  EXPECT_EQ(planner_->getMap().getAllJunctions().size(), 12u);
}

// Test 2: invalid map path throws
TEST(Town01PlannerConstructorTest, InvalidPathThrows)
{
  EXPECT_THROW(mission_planner::MissionPlanner("/nonexistent/hd_map.json"), std::runtime_error);
}

// Test 3: heading-aware snap: -pi/2 heading snaps to lanelet_25_-1
TEST_F(Town01PlannerTest, SnapNorthHeadingPicksCorrectLane)
{
  const hd_map::Lanelet * lane =
    planner_->findNearestLanelet(mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572});

  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->id, "lanelet_25_-1");
}

// Test 4: heading-aware snap: +pi/2 heading snaps to the opposite lane lanelet_25_1
TEST_F(Town01PlannerTest, SnapSouthHeadingPicksOppositeLane)
{
  const hd_map::Lanelet * lane =
    planner_->findNearestLanelet(mission_planner::RoutePoint{158.0, -29.0, 0.0, 1.571});

  ASSERT_NE(lane, nullptr);
  EXPECT_EQ(lane->id, "lanelet_25_1");
}

// Test 5: plan returns a valid route
TEST_F(Town01PlannerTest, PlanReturnsValidRoute)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->empty());
  EXPECT_GT(result->total_length_m, 0.0);
}

// Test 6: route visits expected lanelets in order
TEST_F(Town01PlannerTest, RouteVisitsCorrectLaneletsInOrder)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());
  ASSERT_GE(result->lanelets.size(), 2u);

  EXPECT_EQ(result->lanelets.front().id, "lanelet_25_-1");
  EXPECT_EQ(result->lanelets.back().id, "lanelet_9_1");
}

// Test 7: route contains expected junction lanelet
TEST_F(Town01PlannerTest, JunctionLaneletFlagCorrect)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());

  bool found_junction = false;

  for (const auto & rl : result->lanelets) {
    if (rl.id == "lanelet_333_-1") {
      EXPECT_TRUE(rl.is_junction);
      found_junction = true;
    }
  }

  EXPECT_TRUE(found_junction) << "Junction lanelet lanelet_333_-1 not found in route";
}

// Test 8: all route lanelets contain waypoints
TEST_F(Town01PlannerTest, AllRouteWaypointsNonEmpty)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());

  for (const auto & rl : result->lanelets) {
    EXPECT_FALSE(rl.waypoints.empty()) << "Lanelet " << rl.id << " has no waypoints";
  }
}

// Test 9: total route length matches trimmed route distance
TEST_F(Town01PlannerTest, TotalRouteLengthPlausible)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());

  // Expected trimmed distance for lanelet_25_-1, lanelet_333_-1, lanelet_9_1.
  EXPECT_NEAR(result->total_length_m, 53.85, 1.0);
}

// Test 10: first waypoint is near ego pose after trimming
TEST_F(Town01PlannerTest, StartWaypointNearEgoPose)
{
  const double ego_x = 154.0;
  const double ego_y = -38.0;

  const auto result = planner_->plan(
    mission_planner::RoutePoint{ego_x, ego_y, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->lanelets.empty());
  ASSERT_FALSE(result->lanelets.front().waypoints.empty());

  const auto & first_wp = result->lanelets.front().waypoints.front();

  const double dx = first_wp.x - ego_x;
  const double dy = first_wp.y - ego_y;

  EXPECT_LE(std::sqrt(dx * dx + dy * dy), 3.0)
    << "First waypoint (" << first_wp.x << ", " << first_wp.y << ") is too far from ego (" << ego_x
    << ", " << ego_y << ")";
}

// Test 11: last waypoint is near goal pose after trimming
TEST_F(Town01PlannerTest, LastWaypointNearGoalPose)
{
  const double goal_x = 110.0;
  const double goal_y = -55.5;

  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{goal_x, goal_y, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());
  ASSERT_FALSE(result->lanelets.empty());
  ASSERT_FALSE(result->lanelets.back().waypoints.empty());

  const auto & last_wp = result->lanelets.back().waypoints.back();

  const double dx = last_wp.x - goal_x;
  const double dy = last_wp.y - goal_y;

  EXPECT_LE(std::sqrt(dx * dx + dy * dy), 3.0)
    << "Last waypoint (" << last_wp.x << ", " << last_wp.y << ") is too far from goal (" << goal_x
    << ", " << goal_y << ")";
}

// Test 12: same lanelet route
TEST_F(Town01PlannerTest, StartAndGoalOnSameLanelet)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -15.0, 0.0, -1.572},
    mission_planner::RoutePoint{154.0, -40.0, 0.0, -1.572});

  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->lanelets.size(), 1u);
  EXPECT_EQ(result->lanelets.front().id, "lanelet_25_-1");
}

// Test 13: long road route succeeds
TEST_F(Town01PlannerTest, LongRoadPlanSucceeds)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{396.4, -200.0, 0.0, 1.571},
    mission_planner::RoutePoint{396.4, -50.0, 0.0, 1.571});

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->empty());
  EXPECT_GT(result->total_length_m, 0.0);
}

// Test 14: waypoint headings are in expected map heading range
TEST_F(Town01PlannerTest, WaypointHeadingsInValidRange)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{123.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());

  for (const auto & rl : result->lanelets) {
    for (const auto & wp : rl.waypoints) {
      EXPECT_GE(wp.heading_rad, -M_PI - 0.01) << "Heading below -pi in lanelet " << rl.id;
      EXPECT_LE(wp.heading_rad, M_PI + 0.01) << "Heading above pi in lanelet " << rl.id;
    }
  }
}

// Test 15: longer multi-hop route succeeds
TEST_F(Town01PlannerTest, MultiHopRouteSucceeds)
{
  const auto result = planner_->plan(
    mission_planner::RoutePoint{154.0, -29.0, 0.0, -1.572},
    mission_planner::RoutePoint{318.0, -55.5, 0.0, -M_PI});

  ASSERT_TRUE(result.has_value());
  EXPECT_GE(result->lanelets.size(), 3u);
  EXPECT_GT(result->total_length_m, 50.0);
}

// Driven distance from one lanelet into its successor: connection gap + entered length.
// Re-implemented here on purpose so the oracle below does not share code with the planner.
static double oracleEdgeCost(const hd_map::Lanelet & from, const hd_map::Lanelet & to)
{
  const auto & a = from.centerline.back().position;
  const auto & b = to.centerline.front().position;
  return std::hypot(b.x - a.x, b.y - a.y) + to.length_m;
}

// Plain Dijkstra over the lanelet graph: the reference optimum A* must match.
static double dijkstraCost(const hd_map::HDMap & map, const std::string & from, const std::string & to)
{
  using Item = std::pair<double, std::string>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  std::unordered_map<std::string, double> dist;
  dist[from] = 0.0;
  open.push({0.0, from});
  while (!open.empty()) {
    const auto [d, id] = open.top();
    open.pop();
    if (id == to) {
      return d;
    }
    if (d > dist[id]) {
      continue;
    }
    const hd_map::Lanelet * cur = map.getLaneletById(id);
    for (const hd_map::Lanelet * succ : map.getSuccessors(id)) {
      const double nd = d + oracleEdgeCost(*cur, *succ);
      const auto it = dist.find(succ->id);
      if (it == dist.end() || nd < it->second) {
        dist[succ->id] = nd;
        open.push({nd, succ->id});
      }
    }
  }
  return std::numeric_limits<double>::infinity();
}

// Test 16: A* returns optimal lanelet sequences (same cost as Dijkstra) across the map
TEST_F(Town01PlannerTest, AStarMatchesDijkstraOptimum)
{
  const auto & map = planner_->getMap();
  std::vector<const hd_map::Lanelet *> roads;
  for (const auto & lane : map.getAllLanelets()) {
    if (!lane.is_junction && lane.centerline.size() >= 3) {
      roads.push_back(&lane);
    }
  }

  int compared = 0;
  for (std::size_t i = 0; i < roads.size(); i += 5) {
    for (std::size_t j = 2; j < roads.size(); j += 5) {
      if (i == j) {
        continue;
      }
      const auto & a = roads[i]->centerline[roads[i]->centerline.size() / 2];
      const auto & b = roads[j]->centerline[roads[j]->centerline.size() / 2];
      const auto route = planner_->plan(
        mission_planner::RoutePoint{a.position.x, a.position.y, 0.0, a.heading_rad},
        mission_planner::RoutePoint{b.position.x, b.position.y, 0.0, b.heading_rad});
      if (!route || route->lanelets.size() < 2) {
        continue;
      }

      double astar_cost = 0.0;
      for (std::size_t k = 1; k < route->lanelets.size(); ++k) {
        astar_cost += oracleEdgeCost(
          *map.getLaneletById(route->lanelets[k - 1].id), *map.getLaneletById(route->lanelets[k].id));
      }
      const double best =
        dijkstraCost(map, route->lanelets.front().id, route->lanelets.back().id);
      EXPECT_NEAR(astar_cost, best, 1e-3)
        << route->lanelets.front().id << " -> " << route->lanelets.back().id;
      ++compared;
    }
  }
  std::cout << "[A* vs Dijkstra] compared " << compared << " start/goal pairs\n";
  EXPECT_GT(compared, 50);
}
