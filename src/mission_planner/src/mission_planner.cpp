#include "mission_planner/mission_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace mission_planner
{

MissionPlanner::MissionPlanner(const std::string & hd_map_path)
: map_(std::make_unique<hd_map::HDMap>())
{
  if (!map_->load(hd_map_path)) {
    throw std::runtime_error("[MissionPlanner] Failed to load HD map: " + hd_map_path);
  }
}

std::optional<Route> MissionPlanner::plan(RoutePoint start, RoutePoint goal) const
{
  // Snap start pose: heading from odometry
  const hd_map::Lanelet * start_lane = findNearestLanelet(start);
  if (!start_lane) {
    return std::nullopt;
  }

  // Snap goal pose: heading from RViz2 drag direction
  const hd_map::Lanelet * goal_lane = findNearestLanelet(goal);
  if (!goal_lane) {
    return std::nullopt;
  }

  // Trivial case: already on goal lanelet
  if (start_lane->id == goal_lane->id) {
    Route route = buildRoute({start_lane->id}, {start.x, start.y}, {goal.x, goal.y});
    if (route.empty()) {
      return std::nullopt;
    }
    return route;
  }

  // Run A*
  const std::vector<std::string> ids = runAStar(start_lane->id, goal_lane->id);
  if (ids.empty()) {
    return std::nullopt;
  }

  // Build trimmed route: returns empty if from > to on any lanelet
  Route route = buildRoute(ids, {start.x, start.y}, {goal.x, goal.y});
  if (route.empty()) {
    return std::nullopt;
  }

  return route;
}

const hd_map::Lanelet * MissionPlanner::findNearestLanelet(const RoutePoint & pose) const
{
  const hd_map::Lanelet * best = nullptr;
  double min_dist = std::numeric_limits<double>::max();

  const auto & lanelets = map_->getAllLanelets();
  const double snap_radius_sq = kSnapRadiusM * kSnapRadiusM;

  for (const auto & lane : lanelets) {
    for (const auto & pt : lane.centerline) {
      const double dx = pt.position.x - pose.x;
      const double dy = pt.position.y - pose.y;
      const double dist_sq = dx * dx + dy * dy;

      if (dist_sq <= snap_radius_sq) {
        double heading_diff = std::fmod(std::abs(pt.heading_rad - pose.heading_rad), 2.0 * M_PI);
        if (heading_diff > M_PI) {
          heading_diff = 2.0 * M_PI - heading_diff;
        }

        if (heading_diff <= kHeadingThreshold && dist_sq < min_dist) {
          min_dist = dist_sq;
          best = &lane;
        }
      }
    }
  }

  if (!best) {
    best = map_->getNearestLanelet(pose.x, pose.y);
  }

  return best;
}

std::vector<std::string> MissionPlanner::runAStar(
  const std::string & start_id, const std::string & goal_id) const
{
  const hd_map::Lanelet * goal_lane = map_->getLaneletById(goal_id);
  if (!goal_lane) {
    return {};
  }

  const hd_map::Lanelet * start_lane = map_->getLaneletById(start_id);
  if (!start_lane) {
    return {};
  }

  const Point2D goal_pt = centerlineEnd(*goal_lane);

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

  std::unordered_map<std::string, double> g_cost;
  std::unordered_map<std::string, std::string> came_from;
  std::unordered_set<std::string> closed;

  g_cost[start_id] = 0.0;
  open_set.push({start_id, 0.0, heuristic(*start_lane, goal_pt)});

  while (!open_set.empty()) {
    const AStarNode current = open_set.top();
    open_set.pop();

    if (current.id == goal_id) {
      std::vector<std::string> path;
      for (std::string node = goal_id; node != start_id; ) {
        path.push_back(node);
        auto it = came_from.find(node);
        if (it == came_from.end()) {
          return {};
        }
        node = it->second;
      }
      path.push_back(start_id);
      std::reverse(path.begin(), path.end());
      return path;
    }

    if (!closed.count(current.id)) {
      closed.insert(current.id);

      const hd_map::Lanelet * current_lane = map_->getLaneletById(current.id);
      for (const hd_map::Lanelet * succ : map_->getSuccessors(current.id)) {
        if (succ && current_lane && !closed.count(succ->id)) {
          const double tentative_g = g_cost.at(current.id) + edgeCost(*current_lane, *succ);

          const auto it = g_cost.find(succ->id);
          if (it == g_cost.end() || tentative_g < it->second) {
            g_cost[succ->id] = tentative_g;
            came_from[succ->id] = current.id;

            const double h = heuristic(*succ, goal_pt);
            open_set.push({succ->id, tentative_g, tentative_g + h});
          }
        }
      }
    }
  }

  return {};  // no path found
}

double MissionPlanner::heuristic(const hd_map::Lanelet & lane, Point2D goal_end)
{
  if (lane.centerline.empty()) {
    return 0.0;
  }

  const Point2D end = centerlineEnd(lane);
  return std::hypot(end.x - goal_end.x, end.y - goal_end.y);
}

double MissionPlanner::edgeCost(const hd_map::Lanelet & from, const hd_map::Lanelet & to)
{
  // Driven distance: the connection gap between the two centerlines plus the
  // length of the lanelet being entered.
  double gap = 0.0;
  if (!from.centerline.empty() && !to.centerline.empty()) {
    const auto & a = from.centerline.back().position;
    const auto & b = to.centerline.front().position;
    gap = std::hypot(b.x - a.x, b.y - a.y);
  }
  return gap + to.length_m;
}

Point2D MissionPlanner::centerlineEnd(const hd_map::Lanelet & lane)
{
  if (lane.centerline.empty()) {
    return {0.0, 0.0};
  }
  const auto & pt = lane.centerline.back().position;
  return {pt.x, pt.y};
}

static size_t closestCenterlineIndex(const hd_map::Lanelet & lane, double x, double y)
{
  size_t best_idx = 0;
  double min_dist = std::numeric_limits<double>::max();

  for (size_t i = 0; i < lane.centerline.size(); ++i) {
    const double dx = lane.centerline[i].position.x - x;
    const double dy = lane.centerline[i].position.y - y;
    const double dist = dx * dx + dy * dy;
    if (dist < min_dist) {
      min_dist = dist;
      best_idx = i;
    }
  }
  return best_idx;
}

Route MissionPlanner::buildRoute(
  const std::vector<std::string> & lanelet_ids, Point2D start, Point2D goal) const
{
  Route route;
  route.lanelets.reserve(lanelet_ids.size());

  if (lanelet_ids.empty()) {
    return route;
  }

  const size_t last_idx = lanelet_ids.size() - 1;

  for (size_t i = 0; i < lanelet_ids.size(); ++i) {
    const hd_map::Lanelet * lane = map_->getLaneletById(lanelet_ids[i]);

    // Skip invalid lanelets or malformed map entries.
    if (!lane || lane->centerline.empty()) {
      continue;
    }

    // Trim the first lanelet from the ego position.
    // All middle lanelets start from their first centerline point.
    const size_t from = (i == 0) ? closestCenterlineIndex(*lane, start.x, start.y) : 0;

    // Trim the last lanelet at the goal position.
    // All middle lanelets continue until their last centerline point.
    const size_t to =
      (i == last_idx) ? closestCenterlineIndex(*lane, goal.x, goal.y) : lane->centerline.size() - 1;

    // If the goal/start trimming produces an invalid segment, skip it.
    if (from > to) {
      continue;
    }

    RouteLanelet rl;
    rl.id = lanelet_ids[i];
    rl.is_junction = lane->is_junction;
    rl.waypoints.reserve(to - from + 1);

    // Copy only the selected centerline segment into the route.
    for (size_t j = from; j <= to; ++j) {
      const auto & pt = lane->centerline[j];
      rl.waypoints.push_back({pt.position.x, pt.position.y, pt.position.z, pt.heading_rad});
    }

    // Compute route length using the trimmed segment, not the full lanelet length.
    double segment_length = 0.0;
    for (size_t j = from + 1; j <= to; ++j) {
      const auto & p0 = lane->centerline[j - 1];
      const auto & p1 = lane->centerline[j];

      const double dx = p1.position.x - p0.position.x;
      const double dy = p1.position.y - p0.position.y;
      const double dz = p1.position.z - p0.position.z;

      segment_length += std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    route.total_length_m += segment_length;
    route.lanelets.push_back(std::move(rl));
  }

  return route;
}

}  // namespace mission_planner
