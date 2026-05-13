#include "mission_planner/mission_planner_node.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "planning_msgs/msg/route.hpp"

namespace mission_planner
{

MissionPlannerNode::MissionPlannerNode(const rclcpp::NodeOptions & options)
: Node("mission_planner_node", options), last_odom_(nullptr)
{
  declare_parameter<std::string>("hd_map_path", "");
  const std::string hd_map_path = get_parameter("hd_map_path").as_string();

  if (hd_map_path.empty()) {
    throw std::runtime_error("[MissionPlannerNode] Parameter 'hd_map_path' is not set");
  }

  try {
    planner_ = std::make_unique<MissionPlanner>(hd_map_path);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(), "Failed to initialize MissionPlanner with map '%s': %s", hd_map_path.c_str(),
      e.what());
    throw;
  }

  // Parameterised topic names: consistent with Tasks 3-6 style.
  const auto odometry_topic =
    declare_parameter<std::string>("odometry_topic", "/carla/ego_vehicle/odometry");
  const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
  const auto route_topic = declare_parameter<std::string>("route_topic", "/mission_planner/route");
  const auto route_display_topic =
    declare_parameter<std::string>("route_display_topic", "/mission_planner/route_display");

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic, rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) { onOdometry(msg); });

  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    goal_topic, rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg) { onGoalPose(msg); });

  route_pub_ =
    create_publisher<planning_msgs::msg::Route>(route_topic, rclcpp::QoS(10).transient_local());

  route_display_pub_ =
    create_publisher<nav_msgs::msg::Path>(route_display_topic, rclcpp::QoS(10).transient_local());

  RCLCPP_INFO(get_logger(), "MissionPlannerNode ready: waiting for goal pose");
}

void MissionPlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  last_odom_ = msg;
}

void MissionPlannerNode::onGoalPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr & msg)
{
  if (!last_odom_) {
    RCLCPP_WARN(get_logger(), "No odometry received yet: ignoring goal pose");
    return;
  }

  const double goal_x = msg->pose.position.x;
  const double goal_y = msg->pose.position.y;
  const double goal_yaw = quaternionToYaw(msg->pose.orientation);

  RCLCPP_INFO(
    get_logger(), "Goal received: (%.2f, %.2f) heading: %.2f rad", goal_x, goal_y, goal_yaw);

  planAndPublish(goal_x, goal_y, goal_yaw);
}

void MissionPlannerNode::planAndPublish(double goal_x, double goal_y, double goal_heading_rad)
{
  // Extract start pose from latest odometry.
  const double start_x = last_odom_->pose.pose.position.x;
  const double start_y = last_odom_->pose.pose.position.y;
  const double start_yaw = quaternionToYaw(last_odom_->pose.pose.orientation);

  RCLCPP_INFO(
    get_logger(), "Planning: start (%.2f, %.2f, %.2f rad) → goal (%.2f, %.2f, %.2f rad)", start_x,
    start_y, start_yaw, goal_x, goal_y, goal_heading_rad);

  // Plan the route using planner algorithm (A* search on lanelet graph)
  RoutePoint start{start_x, start_y, 0.0, start_yaw};
  RoutePoint goal{goal_x, goal_y, 0.0, goal_heading_rad};
  const auto result = planner_->plan(start, goal);

  if (!result) {
    RCLCPP_WARN(get_logger(), "Planning failed: no route published");
    return;
  }

  // Convert the route to an enriched route (includes road width, junction flags, curvature)
  // Path Planning must have x,y coordinates, heading, curvature, speed limit, and road/lane width metadata for each waypoint.
  const planning_msgs::msg::Route route_msg = routeToMsg(*result);

  // Publish planning_msgs/Route
  route_pub_->publish(route_msg);

  // Publish nav_msgs/Path for RViz2 visualization (same waypoints, no metadata)
  route_display_pub_->publish(routeToDisplayPath(route_msg));

  RCLCPP_INFO(
    get_logger(), "Route published: %zu waypoints, %.1f m", route_msg.waypoints.size(),
    result->total_length_m);
}

planning_msgs::msg::Route MissionPlannerNode::routeToMsg(const Route & route)
{
  // Convert route data to planning_msgs/Route
  planning_msgs::msg::Route msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "map";
  msg.total_length_m = route.total_length_m;

  // Identify start and goal lanelet IDs
  if (!route.lanelets.empty()) {
    msg.start_lanelet_id = route.lanelets.front().id;
    msg.goal_lanelet_id = route.lanelets.back().id;
  }

  // Pre-allocate waypoints vector for efficiency (total number of waypoints across all lanelets)
  size_t total = 0;
  for (const auto & rl : route.lanelets) {
    total += rl.waypoints.size();
  }
  msg.waypoints.reserve(total);

  // Enrich each waypoint with metadata: road/lane width, junction flag, curvature
  for (const auto & rl : route.lanelets) {
    const size_t n = rl.waypoints.size();

    for (size_t i = 0; i < n; ++i) {
      const auto & wp = rl.waypoints[i];
      planning_msgs::msg::RouteWaypoint rwp;

      rwp.x = wp.x;
      rwp.y = wp.y;
      rwp.z = wp.z;
      rwp.heading = wp.heading_rad;
      rwp.is_junction = rl.is_junction;

      auto [road_half_width, lane_half_width] = calculateWidths(wp.x, wp.y, rl.id);
      rwp.road_half_width = road_half_width;
      rwp.lane_half_width = lane_half_width;

      if (i > 0 && i + 1 < rl.waypoints.size()) {
        rwp.curvature = calculateCurvature(rl.waypoints[i - 1], wp, rl.waypoints[i + 1]);
      } else if (!msg.waypoints.empty()) {
        rwp.curvature = msg.waypoints.back().curvature;
      } else {
        rwp.curvature = 0.0;
      }

      msg.waypoints.push_back(rwp);
    }
  }

  return msg;
}

std::pair<double, double> MissionPlannerNode::calculateWidths(
  double x, double y, const std::string & lanelet_id) const
{
  // O(1) lookup by ID: the waypoint already belongs to this lanelet,
  // so there is no need for an O(n) nearest-lanelet search here.
  const hd_map::Lanelet * lanelet = planner_->getMap().getLaneletById(lanelet_id);

  if (!lanelet || lanelet->left_boundary.empty() || lanelet->right_boundary.empty()) {
    RCLCPP_WARN(
      get_logger(),
      "Waypoint (%.2f, %.2f) has no nearby lanelet: using default widths (road_half_width=%.1f, "
      "lane_half_width=%.1f)",
      x, y, kDefaultRoadHalfWidth, kDefaultRoadHalfWidth / 2.0);
    return {kDefaultRoadHalfWidth, kDefaultRoadHalfWidth / 2.0};
  }

  double left_dist = std::numeric_limits<double>::infinity();
  double right_dist = std::numeric_limits<double>::infinity();

  for (const auto & bp : lanelet->left_boundary) {
    left_dist = std::min(left_dist, std::hypot(bp.x - x, bp.y - y));
  }
  for (const auto & bp : lanelet->right_boundary) {
    right_dist = std::min(right_dist, std::hypot(bp.x - x, bp.y - y));
  }

  const double lane_half_width = std::min(left_dist, right_dist);
  const double road_half_width = std::max(left_dist, right_dist);

  return {road_half_width, lane_half_width};
}

double MissionPlannerNode::calculateCurvature(
  const RoutePoint & prev, const RoutePoint & curr, const RoutePoint & next)
{
  // Three-point curvature estimate:
  // κ = 4A / (a * b * c)
  // where A is the triangle area formed by prev, curr, and next.
  const double ax = prev.x - curr.x;
  const double ay = prev.y - curr.y;
  const double bx = next.x - curr.x;
  const double by = next.y - curr.y;

  // This is twice the signed triangle area.
  const double cross = ax * by - ay * bx;

  const double norm_a = std::hypot(ax, ay);
  const double norm_b = std::hypot(bx, by);
  const double norm_c = std::hypot(next.x - prev.x, next.y - prev.y);

  if (
    norm_a < kMinCurvatureDistance || norm_b < kMinCurvatureDistance ||
    norm_c < kMinCurvatureDistance)
  {
    return 0.0;
  }

  if (std::abs(cross) < kMinCurvatureArea) {
    return 0.0;
  }

  return (2.0 * std::abs(cross)) / (norm_a * norm_b * norm_c);
}

nav_msgs::msg::Path MissionPlannerNode::routeToDisplayPath(const planning_msgs::msg::Route & route)
{
  nav_msgs::msg::Path path;
  path.header = route.header;
  path.poses.reserve(route.waypoints.size());

  for (const auto & wp : route.waypoints) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = route.header;
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.position.z = wp.z;
    ps.pose.orientation.z = std::sin(wp.heading * 0.5);
    ps.pose.orientation.w = std::cos(wp.heading * 0.5);
    path.poses.push_back(ps);
  }

  return path;
}

double MissionPlannerNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  // Standard yaw extraction from quaternion
  // yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace mission_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<mission_planner::MissionPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("mission_planner_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
