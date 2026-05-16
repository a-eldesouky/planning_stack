#include "local_planner/local_planner_node.hpp"

#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "local_planner/cubic_spline.hpp"
#include "planning_msgs/msg/behavior_command.hpp"
#include "planning_msgs/msg/route.hpp"

namespace local_planner
{

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options)
{
  declareParameters();
  loadParams();
  createInterfaces();

  planner_ = std::make_unique<HybridAStar>(vehicle_params_, hybrid_params_);

  ObstacleProcessorParams obs_params;
  obs_params.lookahead = obstacle_lookahead_;
  obs_params.inflation = obstacle_inflation_;
  obs_params.corridor_width_multiplier = corridor_width_multiplier_;
  obs_params.avoidance_corridor_multiplier = avoidance_corridor_multiplier_;
  obs_params.min_avoidance_corridor_half_width = min_avoidance_corridor_half_width_;
  obs_params.ego_filter_multiplier = ego_filter_multiplier_;
  obstacle_processor_ = std::make_unique<ObstacleProcessor>(obs_params, vehicle_params_);

  avoidance_latch_ = std::make_unique<AvoidanceLatch>(avoidance_release_distance_);
  stop_latch_ = std::make_unique<StopLatch>(stop_commit_distance_, stop_creep_speed_);
}

void LocalPlannerNode::declareParameters()
{
  declare_parameter("vehicle.lf", 0.7725);
  declare_parameter("vehicle.lr", 0.7725);
  declare_parameter("vehicle.wheelbase", 1.545);
  declare_parameter("vehicle.width", 1.45);
  declare_parameter("vehicle.length", 2.3);
  declare_parameter("vehicle.max_steering_angle", 0.2792);
  declare_parameter("vehicle.max_steering_rate", 0.17);
  declare_parameter("vehicle.min_turning_radius", 5.39);

  declare_parameter("input.behavior_command_topic", "/behavior_planner/behavior_command");
  declare_parameter("behavior.stop_creep_speed", 0.5);
  declare_parameter("behavior.stop_creep_distance", 2.0);
  declare_parameter("behavior.stop_commit_distance", 4.0);

  declare_parameter("input.route_topic", "/mission_planner/route");
  declare_parameter("input.odometry_topic", "/carla/ego_vehicle/odometry");
  declare_parameter("input.objects_topic", "/fake/objects");

  declare_parameter("output.trajectory_topic", "/local_planner/trajectory");
  declare_parameter("output.debug_trajectory_topic", "/local_planner/debug/trajectory");
  declare_parameter("output.publish_rate", 10.0);
  declare_parameter("output.frame_id", "map");

  declare_parameter("path_following.enable_smoothing", true);
  declare_parameter("path_following.lookahead_distance", 30.0);
  declare_parameter("path_following.min_waypoint_spacing", 0.5);
  declare_parameter("path_following.output_points", 100);

  declare_parameter("path_trim.search_window_size", 100);
  declare_parameter("path_trim.look_behind_points", 2);
  declare_parameter("path_trim.max_cross_track_error", 3.0);
  declare_parameter("path_trim.goal_reached_threshold", 1.5);

  declare_parameter("obstacle_detection.lookahead", 30.0);
  declare_parameter("obstacle_detection.inflation", 0.8);
  declare_parameter("obstacle_detection.corridor_width_multiplier", 1.5);
  declare_parameter("obstacle_detection.ego_filter_multiplier", 0.5);
  declare_parameter("obstacle_detection.road_boundary_margin", 0.5);
  declare_parameter("obstacle_detection.avoidance_corridor_multiplier", 3.0);
  declare_parameter("obstacle_detection.min_avoidance_corridor_half_width", 5.2);
  declare_parameter("obstacle_detection.avoidance_release_distance", 6.0);

  declare_parameter("hybrid_astar.grid_resolution", 0.5);
  declare_parameter("hybrid_astar.heading_bins", 72);
  declare_parameter("hybrid_astar.steering_samples", 11);
  declare_parameter("hybrid_astar.step_size", 0.8);
  declare_parameter("hybrid_astar.max_iterations", 15000);
  declare_parameter("hybrid_astar.obstacle_inflation", 0.8);
  declare_parameter("hybrid_astar.obstacle_lookahead", 20.0);
  declare_parameter("hybrid_astar.heuristic_weight", 1.2);
  declare_parameter("hybrid_astar.goal_position_tolerance", 1.5);
  declare_parameter("hybrid_astar.goal_heading_tolerance", 0.35);
}

void LocalPlannerNode::loadParams()
{
  vehicle_params_.lf = get_parameter("vehicle.lf").as_double();
  vehicle_params_.lr = get_parameter("vehicle.lr").as_double();
  vehicle_params_.wheelbase = get_parameter("vehicle.wheelbase").as_double();
  vehicle_params_.width = get_parameter("vehicle.width").as_double();
  vehicle_params_.length = get_parameter("vehicle.length").as_double();
  vehicle_params_.max_steering_angle = get_parameter("vehicle.max_steering_angle").as_double();
  vehicle_params_.max_steering_rate = get_parameter("vehicle.max_steering_rate").as_double();
  vehicle_params_.min_turning_radius = get_parameter("vehicle.min_turning_radius").as_double();

  hybrid_params_.grid_resolution = get_parameter("hybrid_astar.grid_resolution").as_double();
  hybrid_params_.heading_bins = get_parameter("hybrid_astar.heading_bins").as_int();
  hybrid_params_.steering_samples = get_parameter("hybrid_astar.steering_samples").as_int();
  hybrid_params_.step_size = get_parameter("hybrid_astar.step_size").as_double();
  hybrid_params_.max_iterations = get_parameter("hybrid_astar.max_iterations").as_int();
  hybrid_params_.obstacle_inflation = get_parameter("hybrid_astar.obstacle_inflation").as_double();
  hybrid_params_.obstacle_lookahead = get_parameter("hybrid_astar.obstacle_lookahead").as_double();
  hybrid_params_.heuristic_weight = get_parameter("hybrid_astar.heuristic_weight").as_double();
  hybrid_params_.goal_position_tolerance =
    get_parameter("hybrid_astar.goal_position_tolerance").as_double();
  hybrid_params_.goal_heading_tolerance =
    get_parameter("hybrid_astar.goal_heading_tolerance").as_double();

  enable_path_smoothing_ = get_parameter("path_following.enable_smoothing").as_bool();
  lookahead_distance_ = get_parameter("path_following.lookahead_distance").as_double();
  min_waypoint_spacing_ = get_parameter("path_following.min_waypoint_spacing").as_double();
  output_points_ = static_cast<std::size_t>(get_parameter("path_following.output_points").as_int());

  search_window_size_ =
    static_cast<std::size_t>(get_parameter("path_trim.search_window_size").as_int());
  look_behind_points_ =
    static_cast<std::size_t>(get_parameter("path_trim.look_behind_points").as_int());
  max_cross_track_error_ = get_parameter("path_trim.max_cross_track_error").as_double();
  goal_reached_threshold_ = get_parameter("path_trim.goal_reached_threshold").as_double();

  obstacle_lookahead_ = get_parameter("obstacle_detection.lookahead").as_double();
  obstacle_inflation_ = get_parameter("obstacle_detection.inflation").as_double();
  corridor_width_multiplier_ =
    get_parameter("obstacle_detection.corridor_width_multiplier").as_double();
  ego_filter_multiplier_ = get_parameter("obstacle_detection.ego_filter_multiplier").as_double();
  road_boundary_margin_ = get_parameter("obstacle_detection.road_boundary_margin").as_double();
  avoidance_corridor_multiplier_ =
    get_parameter("obstacle_detection.avoidance_corridor_multiplier").as_double();
  min_avoidance_corridor_half_width_ =
    get_parameter("obstacle_detection.min_avoidance_corridor_half_width").as_double();
  avoidance_release_distance_ =
    get_parameter("obstacle_detection.avoidance_release_distance").as_double();

  behavior_command_topic_ = get_parameter("input.behavior_command_topic").as_string();
  stop_creep_speed_ = get_parameter("behavior.stop_creep_speed").as_double();
  stop_creep_distance_ = get_parameter("behavior.stop_creep_distance").as_double();
  stop_commit_distance_ = get_parameter("behavior.stop_commit_distance").as_double();

  current_speed_limit_ = kFallbackSpeed;
  frame_id_ = get_parameter("output.frame_id").as_string();
  trajectory_topic_ = get_parameter("output.trajectory_topic").as_string();
  debug_topic_ = get_parameter("output.debug_trajectory_topic").as_string();
}

void LocalPlannerNode::createInterfaces()
{
  const auto route_topic = get_parameter("input.route_topic").as_string();
  const auto odom_topic = get_parameter("input.odometry_topic").as_string();
  const auto objects_topic = get_parameter("input.objects_topic").as_string();

  route_sub_ = create_subscription<planning_msgs::msg::Route>(
    route_topic, rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&LocalPlannerNode::onRoute, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, rclcpp::QoS(10).best_effort(),
    std::bind(&LocalPlannerNode::onOdometry, this, std::placeholders::_1));

  objects_sub_ = create_subscription<derived_object_msgs::msg::ObjectArray>(
    objects_topic, rclcpp::QoS(10).best_effort(),
    std::bind(&LocalPlannerNode::onObjects, this, std::placeholders::_1));

  behavior_cmd_sub_ = create_subscription<planning_msgs::msg::BehaviorCommand>(
    behavior_command_topic_, rclcpp::QoS(10).reliable(),
    std::bind(&LocalPlannerNode::onBehaviorCommand, this, std::placeholders::_1));

  // Main output: planning_msgs/Trajectory
  trajectory_pub_ =
    create_publisher<planning_msgs::msg::Trajectory>(trajectory_topic_, rclcpp::QoS(1).reliable());

  // Debug: nav_msgs/Path for RViz2 (geometry only)
  debug_pub_ = create_publisher<nav_msgs::msg::Path>(debug_topic_, rclcpp::QoS(1).reliable());

  const double rate_hz = get_parameter("output.publish_rate").as_double();
  const double safe_rate_hz = rate_hz > 0.0 ? rate_hz : 10.0;

  if (rate_hz <= 0.0) {
    RCLCPP_WARN(
      get_logger(), "Invalid output.publish_rate %.2f. Falling back to 10.0 Hz.", rate_hz);
  }

  const auto period = std::chrono::milliseconds(static_cast<long>(1000.0 / safe_rate_hz));
  publish_timer_ = create_wall_timer(period, std::bind(&LocalPlannerNode::onPublishTimer, this));
}

void LocalPlannerNode::onRoute(const planning_msgs::msg::Route::ConstSharedPtr & msg)
{
  if (msg->waypoints.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty route: ignoring");
    return;
  }

  current_route_ = msg;
  last_closest_idx_ = 0;
  current_trajectory_.clear();

  avoidance_latch_->reset();

  RCLCPP_INFO(
    get_logger(), "New route received: %zu waypoints, %.1f m [%s → %s]", msg->waypoints.size(),
    msg->total_length_m, msg->start_lanelet_id.c_str(), msg->goal_lanelet_id.c_str());
}

void LocalPlannerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  last_odom_ = msg;
}

void LocalPlannerNode::onObjects(const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg)
{
  if (!last_odom_) {
    return;
  }

  const double ego_x = last_odom_->pose.pose.position.x;
  const double ego_y = last_odom_->pose.pose.position.y;
  const double ego_yaw = quaternionToYaw(last_odom_->pose.pose.orientation);

  obstacle_processor_->update(*msg, ego_x, ego_y, ego_yaw);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000, "[ObjectsDebug] kept_obstacles=%zu",
    obstacle_processor_->obstacles().size());
}

void LocalPlannerNode::onBehaviorCommand(
  const planning_msgs::msg::BehaviorCommand::ConstSharedPtr & msg)
{
  using BC = planning_msgs::msg::BehaviorCommand;

  current_behavior_ = *msg;
  behavior_received_ = true;

  switch (msg->behavior) {
    case BC::STOP_AT_TRAFFIC_LIGHT:
    case BC::WAIT_BEHIND_OBSTACLE:
      current_speed_limit_ =
        (msg->target_speed > 0.0f) ? static_cast<double>(msg->target_speed) : stop_creep_speed_;
      break;
    default:
      current_speed_limit_ =
        (msg->target_speed > 0.0f) ? static_cast<double>(msg->target_speed) : kFallbackSpeed;
      break;
  }
}

void LocalPlannerNode::onPublishTimer()
{
  // Recompute the trajectory on every timer tick to ensure the latest route, odometry, obstacles,
  // and behavior commands are used.
  computeTrajectory();

  // Do not publish empty trajectories
  if (current_trajectory_.empty()) {
    return;
  }

  const rclcpp::Time stamp = get_clock()->now();
  trajectory_pub_->publish(toTrajectory(current_trajectory_, stamp));
  debug_pub_->publish(toNavPath(current_trajectory_, stamp));
}

void LocalPlannerNode::computeTrajectory()
{
  if (!current_route_ || !last_odom_) {
    return;
  }

  const Pose ego = getCurrentEgoPose();
  const bool currently_stopping = isStoppingBehavior();

  if (stop_latch_->isCommitted()) {
    double ego_speed = 0.0;
    if (last_odom_) {
      const auto & twist = last_odom_->twist.twist.linear;
      ego_speed = std::hypot(twist.x, twist.y);
    }

    if (stop_latch_->update(
          ego, currently_stopping, ego_speed, current_trajectory_, *get_clock(), get_logger()))
    {
      return;
    }
  }

  if (handleGoalReached(ego)) {
    return;
  }

  const auto & waypoints = current_route_->waypoints;
  const std::size_t closest_idx = findClosestWaypoint(ego.x, ego.y);
  last_closest_idx_ = closest_idx;

  const double cross_track =
    std::hypot(ego.x - waypoints[closest_idx].x, ego.y - waypoints[closest_idx].y);
  if (cross_track > max_cross_track_error_) {
    RCLCPP_WARN(
      get_logger(), "Cross-track error %.2f m exceeds threshold %.2f m", cross_track,
      max_cross_track_error_);
  }

  if (isAvoidanceRequested()) {
    if (computeAvoidanceTrajectory(ego, closest_idx)) {
      return;
    }

    publishFallbackStopTrajectory(ego, "hybrid_astar_failed");
    return;
  }

  // Behavior is no longer avoidance: release the avoidance latch if held.
  if (avoidance_latch_->isLatched()) {
    avoidance_latch_->reset();
    RCLCPP_INFO(get_logger(), "[Avoidance] Behavior cleared: releasing avoidance path latch");
  }

  if (current_behavior_.behavior == planning_msgs::msg::BehaviorCommand::WAIT_BEHIND_OBSTACLE) {
    publishFallbackStopTrajectory(ego, "wait_behind_obstacle");
    return;
  }

  current_trajectory_ = buildInterpolatedPath(closest_idx);

  if (currently_stopping) {
    using BC = planning_msgs::msg::BehaviorCommand;
    const std::string reason =
      current_behavior_.behavior == BC::WAIT_BEHIND_OBSTACLE
      ? "obstacle_wait_committed"
      : "traffic_light_committed";
    stop_latch_->maybeCommit(ego, current_trajectory_, get_logger(), reason);
  }
}

bool LocalPlannerNode::isStoppingBehavior() const
{
  using BC = planning_msgs::msg::BehaviorCommand;
  return behavior_received_ && (current_behavior_.behavior == BC::STOP_AT_TRAFFIC_LIGHT ||
                                current_behavior_.behavior == BC::WAIT_BEHIND_OBSTACLE);
}

bool LocalPlannerNode::isAvoidanceRequested() const
{
  using BC = planning_msgs::msg::BehaviorCommand;
  return behavior_received_ && current_behavior_.behavior == BC::AVOID_STATIC_OBSTACLE;
}

bool LocalPlannerNode::handleGoalReached(const Pose & ego)
{
  const auto & waypoints = current_route_->waypoints;
  const double dist_to_goal = std::hypot(ego.x - waypoints.back().x, ego.y - waypoints.back().y);

  if (dist_to_goal >= goal_reached_threshold_) {
    return false;
  }

  RCLCPP_INFO(get_logger(), "Goal reached (%.2f m): publishing stop trajectory", dist_to_goal);

  TrajectoryPoint hold_pt;
  hold_pt.pose = ego;
  hold_pt.max_velocity_mps = 0.0;
  hold_pt.is_stop_point = false;

  current_trajectory_ = {hold_pt, makeStopPoint(ego, "goal")};
  current_route_ = nullptr;
  return true;
}

bool LocalPlannerNode::computeAvoidanceTrajectory(const Pose & ego, std::size_t closest_idx)
{
  const auto & obstacles = obstacle_processor_->obstacles();

  // Follow the latched avoidance path if still valid
  if (avoidance_latch_->isLatched()) {
    const auto trimmed = avoidance_latch_->trimFromEgo(ego);

    if (!trimmed.empty() && !avoidance_latch_->isObstacleCleared(ego)) {
      current_trajectory_ = trimmed;
      RCLCPP_DEBUG(
        get_logger(), "[Avoidance] Following latched path: %zu pts remaining",
        current_trajectory_.size());
      return true;
    }

    // Obstacle cleared: release latch and resume normal following.
    avoidance_latch_->reset();
    RCLCPP_INFO(get_logger(), "[Avoidance] Obstacle cleared: releasing avoidance path latch");
    current_trajectory_ = buildInterpolatedPath(closest_idx);
    return true;
  }

  // Guard: no usable obstacle geometry
  if (obstacles.empty() || !obstacle_processor_->obstacleAhead(ego, isAvoidanceRequested())) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Avoidance requested but no valid obstacle geometry: stopping safely");
    publishFallbackStopTrajectory(ego, "obstacle_geometry_unavailable");
    return true;
  }

  // Goal selection
  const auto & waypoints = current_route_->waypoints;

  double accumulated = 0.0;
  std::size_t goal_idx = closest_idx;
  for (std::size_t i = closest_idx + 1; i < waypoints.size() && accumulated < lookahead_distance_;
       ++i) {
    const double dx = waypoints[i].x - waypoints[i - 1].x;
    const double dy = waypoints[i].y - waypoints[i - 1].y;
    accumulated += std::hypot(dx, dy);
    goal_idx = i;
  }

  // Push goal forward past any obstacle circle.
  for (std::size_t i = goal_idx; i < waypoints.size(); ++i) {
    const auto & gwp = waypoints[i];
    bool goal_clear = true;
    for (const auto & obs : obstacles) {
      if (std::hypot(gwp.x - obs.x, gwp.y - obs.y) < obs.radius + vehicle_params_.length) {
        goal_clear = false;
        break;
      }
    }
    if (goal_clear) {
      goal_idx = i;
      break;
    }
  }

  const auto & goal_wp = waypoints[goal_idx];
  const Pose goal{goal_wp.x, goal_wp.y, goal_wp.z, goal_wp.heading};
  const auto corridor = buildRouteCorridor(closest_idx, goal_idx);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[AvoidanceDebug] obstacles=%zu start=(%.2f, %.2f, %.2f) goal=(%.2f, %.2f, %.2f) "
    "corridor_half_width=%.2f",
    obstacles.size(), ego.x, ego.y, ego.heading, goal.x, goal.y, goal.heading, corridor.half_width);

  const auto result = planner_->plan(ego, goal, obstacles, &corridor, current_speed_limit_);

  if (!result.success) {
    const bool limit_hit = avoidance_latch_->recordFailure(get_logger());
    RCLCPP_WARN(
      get_logger(), "Hybrid A* failed: %s: remaining behind obstacle (%d/%d)",
      result.reason.c_str(), avoidance_latch_->consecutiveFailures(), AvoidanceLatch::kMaxFailures);
    (void)limit_hit;
    return false;
  }

  avoidance_latch_->resetFailureCount();
  avoidance_latch_->latch(result.path, obstacles, ego);

  RCLCPP_INFO(
    get_logger(), "[Avoidance] Path latched: %zu points, release when %.1fm past front circle",
    result.path.size(), avoidance_release_distance_);

  // Smoothing is intentionally disabled for avoidance paths: the spline may
  // cut through inflated obstacle circles while fitting the lateral bend.
  current_trajectory_ = result.path;
  return true;
}

std::vector<TrajectoryPoint> LocalPlannerNode::buildInterpolatedPath(std::size_t closest_idx) const
{
  using BC = planning_msgs::msg::BehaviorCommand;

  const bool is_traffic_light_stop =
    behavior_received_ && current_behavior_.behavior == BC::STOP_AT_TRAFFIC_LIGHT;
  const bool is_obstacle_stop =
    behavior_received_ && current_behavior_.behavior == BC::WAIT_BEHIND_OBSTACLE;
  const bool is_stopping = is_traffic_light_stop || is_obstacle_stop;

  const double horizon = getPlanningHorizon(is_stopping);
  auto path = buildRouteFollowingPath(closest_idx, horizon);

  if (path.empty()) {
    return path;
  }

  if (is_stopping) {
    applyStopConstraint(path, is_traffic_light_stop);
  }

  return smoothRouteFollowingPath(std::move(path), is_stopping, is_traffic_light_stop);
}

double LocalPlannerNode::getPlanningHorizon(bool is_stopping) const
{
  if (!is_stopping) {
    return lookahead_distance_;
  }

  const double requested = static_cast<double>(current_behavior_.safe_distance);
  if (std::isfinite(requested) && requested > 0.5) {
    return std::min(lookahead_distance_, requested);
  }

  return std::min(lookahead_distance_, stop_creep_distance_);
}

std::vector<TrajectoryPoint> LocalPlannerNode::buildRouteFollowingPath(
  std::size_t closest_idx, double planning_horizon) const
{
  const auto & waypoints = current_route_->waypoints;

  std::vector<TrajectoryPoint> path;
  path.reserve(output_points_);

  double accumulated = 0.0;
  double prev_x = waypoints[closest_idx].x;
  double prev_y = waypoints[closest_idx].y;

  for (std::size_t i = closest_idx; i < waypoints.size(); ++i) {
    const auto & wp = waypoints[i];

    const double dx = wp.x - prev_x;
    const double dy = wp.y - prev_y;
    const double dist = std::hypot(dx, dy);

    if (i > closest_idx && dist < min_waypoint_spacing_) {
      continue;
    }

    accumulated += dist;
    if (accumulated > planning_horizon) {
      break;
    }

    TrajectoryPoint pt;
    pt.pose.x = wp.x;
    pt.pose.y = wp.y;
    pt.pose.z = wp.z;
    pt.pose.heading = wp.heading;
    pt.curvature = wp.curvature;
    pt.max_velocity_mps = static_cast<float>(current_speed_limit_);
    pt.is_stop_point = false;

    path.push_back(pt);

    prev_x = wp.x;
    prev_y = wp.y;

    if (path.size() >= output_points_) {
      break;
    }
  }

  return path;
}

void LocalPlannerNode::applyStopConstraint(
  std::vector<TrajectoryPoint> & path, bool is_traffic_light_stop) const
{
  if (path.empty()) {
    return;
  }

  const auto stop_speed_cap = static_cast<decltype(path.front().max_velocity_mps)>(
    std::max(0.0, std::min(current_speed_limit_, stop_creep_speed_)));

  for (auto & pt : path) {
    pt.max_velocity_mps = std::min(pt.max_velocity_mps, stop_speed_cap);
  }

  auto & stop_pt = path.back();
  stop_pt.max_velocity_mps = 0.0;
  stop_pt.is_stop_point = true;
  stop_pt.stop_reason = is_traffic_light_stop ? "traffic_light" : "obstacle";

  double dist_to_stop = 0.0;
  for (int j = static_cast<int>(path.size()) - 2; j >= 0; --j) {
    const double dx = path[j + 1].pose.x - path[j].pose.x;
    const double dy = path[j + 1].pose.y - path[j].pose.y;
    dist_to_stop += std::hypot(dx, dy);

    if (dist_to_stop <= stop_creep_distance_) {
      path[j].max_velocity_mps = std::min(path[j].max_velocity_mps, stop_speed_cap);
    } else {
      break;
    }
  }
}

std::vector<TrajectoryPoint> LocalPlannerNode::smoothRouteFollowingPath(
  std::vector<TrajectoryPoint> path, bool is_stopping, bool is_traffic_light_stop) const
{
  if (!enable_path_smoothing_ || path.size() < 3) {
    return path;
  }

  const auto spline = CubicSpline::fit(path);
  if (!spline.valid()) {
    return path;
  }

  auto smoothed = spline.resample(min_waypoint_spacing_);
  if (smoothed.empty()) {
    return path;
  }

  if (is_stopping) {
    applyStopConstraint(smoothed, is_traffic_light_stop);
  }

  return smoothed;
}

Pose LocalPlannerNode::getCurrentEgoPose() const
{
  return Pose{
    last_odom_->pose.pose.position.x, last_odom_->pose.pose.position.y,
    last_odom_->pose.pose.position.z, quaternionToYaw(last_odom_->pose.pose.orientation)};
}

std::size_t LocalPlannerNode::findClosestWaypoint(double ego_x, double ego_y) const
{
  const auto & waypoints = current_route_->waypoints;
  const std::size_t n = waypoints.size();
  const std::size_t start =
    (last_closest_idx_ > look_behind_points_) ? last_closest_idx_ - look_behind_points_ : 0;
  const std::size_t end = std::min(n, start + search_window_size_);

  double best_dist = std::numeric_limits<double>::infinity();
  std::size_t best_idx = last_closest_idx_;

  for (std::size_t i = start; i < end; ++i) {
    const double d = std::hypot(ego_x - waypoints[i].x, ego_y - waypoints[i].y);
    if (d < best_dist) {
      best_dist = d;
      best_idx = i;
    }
  }

  return best_idx;
}

RouteCorridor LocalPlannerNode::buildRouteCorridor(std::size_t start_idx, std::size_t end_idx) const
{
  const double fallback = vehicle_params_.width * corridor_width_multiplier_;

  RouteCorridor corridor;
  corridor.half_width = fallback;

  if (!current_route_ || current_route_->waypoints.empty()) {
    return corridor;
  }

  const auto & waypoints = current_route_->waypoints;
  start_idx = std::min(start_idx, waypoints.size() - 1);
  end_idx = std::min(end_idx, waypoints.size() - 1);
  if (end_idx < start_idx) {
    std::swap(start_idx, end_idx);
  }

  const bool is_avoidance =
    (current_behavior_.behavior == planning_msgs::msg::BehaviorCommand::AVOID_STATIC_OBSTACLE);
  const double width_scale = is_avoidance ? avoidance_corridor_multiplier_ : 1.0;

  corridor.centerline.reserve(end_idx - start_idx + 1);
  double max_w = 0.0;

  for (std::size_t i = start_idx; i <= end_idx; ++i) {
    const auto & wp = waypoints[i];
    const double base_w = (wp.road_half_width > 0.01)
                            ? std::max(0.0, wp.road_half_width - road_boundary_margin_)
                            : fallback;

    double pw = base_w * width_scale;
    if (is_avoidance) {
      pw = std::max(pw, min_avoidance_corridor_half_width_);
    }

    corridor.centerline.push_back({wp.x, wp.y, pw});
    max_w = std::max(max_w, pw);
  }

  corridor.half_width = (max_w > 0.01) ? max_w : fallback;
  return corridor;
}

TrajectoryPoint LocalPlannerNode::makeStopPoint(const Pose & pose, const std::string & reason) const
{
  TrajectoryPoint stop_pt;
  stop_pt.pose = pose;
  stop_pt.max_velocity_mps = 0.0;
  stop_pt.is_stop_point = true;
  stop_pt.stop_reason = reason;
  return stop_pt;
}

void LocalPlannerNode::publishFallbackStopTrajectory(const Pose & ego, const std::string & reason)
{
  TrajectoryPoint hold_pt;
  hold_pt.pose = ego;
  hold_pt.max_velocity_mps = 0.0;
  hold_pt.is_stop_point = false;
  hold_pt.stop_reason = reason;

  TrajectoryPoint stop_pt = hold_pt;
  stop_pt.is_stop_point = true;

  // Two points so downstream trimming never receives a one-point trajectory.
  current_trajectory_ = {hold_pt, stop_pt};

  RCLCPP_WARN_THROTTLE(
    get_logger(), *get_clock(), 1000, "[LocalPlanner] Fallback stop trajectory published: %s",
    reason.c_str());
}

planning_msgs::msg::Trajectory LocalPlannerNode::toTrajectory(
  const std::vector<TrajectoryPoint> & points, const rclcpp::Time & stamp) const
{
  planning_msgs::msg::Trajectory traj;
  traj.header.stamp = stamp;
  traj.header.frame_id = frame_id_;
  traj.points.reserve(points.size());

  for (const auto & pt : points) {
    planning_msgs::msg::TrajectoryPoint tp;
    tp.pose.position.x = pt.pose.x;
    tp.pose.position.y = pt.pose.y;
    tp.pose.position.z = pt.pose.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.pose.heading);
    tp.pose.orientation = tf2::toMsg(q);

    tp.max_velocity_mps = pt.max_velocity_mps;
    tp.is_stop_point = pt.is_stop_point;
    tp.stop_reason = pt.stop_reason;

    traj.points.push_back(tp);
  }

  return traj;
}

nav_msgs::msg::Path LocalPlannerNode::toNavPath(
  const std::vector<TrajectoryPoint> & points, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = frame_id_;
  path.poses.reserve(points.size());

  for (const auto & pt : points) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp = stamp;
    ps.header.frame_id = frame_id_;
    ps.pose.position.x = pt.pose.x;
    ps.pose.position.y = pt.pose.y;
    ps.pose.position.z = pt.pose.z;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pt.pose.heading);
    ps.pose.orientation = tf2::toMsg(q);

    path.poses.push_back(ps);
  }

  return path;
}

double LocalPlannerNode::quaternionToYaw(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<local_planner::LocalPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("local_planner_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
