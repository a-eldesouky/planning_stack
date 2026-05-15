#include "behavior_planner/behavior_planner_node.hpp"

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <exception>
#include <functional>
#include <mutex>
#include <utility>

#include "behavior_planner/actions.hpp"
#include "behavior_planner/conditions.hpp"

using namespace std::chrono_literals;

namespace behavior_planner
{

BehaviorPlannerNode::BehaviorPlannerNode(const rclcpp::NodeOptions & options)
: Node("behavior_planner_node", options), ctx_(std::make_shared<BehaviorContext>())
{
  declareAndLoadParams();
  createSubscriptions();
  createPublisher();
  registerBTNodes();
  loadBehaviorTree();
  createTickTimer();

  RCLCPP_INFO(get_logger(), "BehaviorPlannerNode ready");
}

void BehaviorPlannerNode::declareAndLoadParams()
{
  // Tick rate.
  declare_parameter<double>("tick_rate_hz", 10.0);

  // Topic parameters.
  declare_parameter<std::string>("global_route_topic", "/mission_planner/route");
  declare_parameter<std::string>("odometry_topic", "/carla/ego_vehicle/odometry");
  declare_parameter<std::string>("objects_topic", "/fake/objects");
  declare_parameter<std::string>("traffic_light_status_topic", "/carla/traffic_lights/status");
  declare_parameter<std::string>("traffic_light_info_topic", "/carla/traffic_lights/info");
  declare_parameter<std::string>("behavior_command_topic", "/behavior_planner/behavior_command");

  // Behavior tree XML path. Can be absolute or relative to package share.
  declare_parameter<std::string>("behavior_tree_file", "behavior_trees/main_tree.xml");

  // Optional Groot2 live monitoring.
  declare_parameter<bool>("enable_groot_monitoring", true);

  // Behavior parameters.
  declare_parameter<double>("normal_speed", 8.0);
  declare_parameter<double>("following_speed", 4.0);
  declare_parameter<double>("avoidance_speed", 3.0);
  declare_parameter<double>("leading_vehicle_detection_distance", 30.0);
  declare_parameter<double>("safe_following_distance", 10.0);
  declare_parameter<double>("route_lateral_threshold", 3.5);
  declare_parameter<double>("static_speed_threshold", 0.5);
  declare_parameter<double>("ego_filter_radius", 2.5);
  declare_parameter<double>("junction_detection_distance", 15.0);
  declare_parameter<double>("traffic_light_stop_distance", 12.0);
  declare_parameter<bool>("enable_avoidance", true);
  declare_parameter<double>("minimum_avoidance_distance", 5.0);
  declare_parameter<double>("road_boundary_margin", 0.5);
  declare_parameter<double>("goal_reached_threshold", 2.0);

  // Hysteresis parameters.
  declare_parameter<int>("leading_vehicle_confirm_ticks", 1);
  declare_parameter<int>("static_obstacle_confirm_ticks", 2);
  declare_parameter<int>("traffic_light_confirm_ticks", 2);
  declare_parameter<int>("exit_hysteresis_ticks", 3);

  // Set topic parameters.
  global_route_topic_ = get_parameter("global_route_topic").as_string();
  odometry_topic_ = get_parameter("odometry_topic").as_string();
  objects_topic_ = get_parameter("objects_topic").as_string();
  tl_status_topic_ = get_parameter("traffic_light_status_topic").as_string();
  tl_info_topic_ = get_parameter("traffic_light_info_topic").as_string();
  behavior_command_topic_ = get_parameter("behavior_command_topic").as_string();
  behavior_tree_file_ = get_parameter("behavior_tree_file").as_string();
  enable_groot_monitoring_ = get_parameter("enable_groot_monitoring").as_bool();

  auto & params = ctx_->params;
  params.normal_speed = get_parameter("normal_speed").as_double();
  params.following_speed = get_parameter("following_speed").as_double();
  params.avoidance_speed = get_parameter("avoidance_speed").as_double();
  params.leading_vehicle_detection_distance =
    get_parameter("leading_vehicle_detection_distance").as_double();
  params.safe_following_distance = get_parameter("safe_following_distance").as_double();
  params.route_lateral_threshold = get_parameter("route_lateral_threshold").as_double();
  params.static_speed_threshold = get_parameter("static_speed_threshold").as_double();
  params.ego_filter_radius = get_parameter("ego_filter_radius").as_double();
  params.junction_detection_distance = get_parameter("junction_detection_distance").as_double();
  params.traffic_light_stop_distance = get_parameter("traffic_light_stop_distance").as_double();
  params.enable_avoidance = get_parameter("enable_avoidance").as_bool();
  params.minimum_avoidance_distance = get_parameter("minimum_avoidance_distance").as_double();
  params.road_boundary_margin = get_parameter("road_boundary_margin").as_double();
  params.goal_reached_threshold = get_parameter("goal_reached_threshold").as_double();

  auto & hysteresis = ctx_->hysteresis;
  hysteresis.leading_vehicle_confirm_ticks =
    get_parameter("leading_vehicle_confirm_ticks").as_int();
  hysteresis.static_obstacle_confirm_ticks =
    get_parameter("static_obstacle_confirm_ticks").as_int();
  hysteresis.traffic_light_confirm_ticks = get_parameter("traffic_light_confirm_ticks").as_int();
  hysteresis.exit_hysteresis_ticks = get_parameter("exit_hysteresis_ticks").as_int();

  RCLCPP_INFO(
    get_logger(), "Behavior planner params loaded. Groot2 monitoring: %s",
    enable_groot_monitoring_ ? "enabled" : "disabled");
}

void BehaviorPlannerNode::createSubscriptions()
{
  using std::placeholders::_1;

  route_sub_ = create_subscription<planning_msgs::msg::Route>(
    global_route_topic_, rclcpp::QoS(10).transient_local(),
    std::bind(&BehaviorPlannerNode::routeCallback, this, _1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, rclcpp::QoS(10), std::bind(&BehaviorPlannerNode::odometryCallback, this, _1));

  objects_sub_ = create_subscription<derived_object_msgs::msg::ObjectArray>(
    objects_topic_, rclcpp::QoS(10), std::bind(&BehaviorPlannerNode::objectsCallback, this, _1));

  tl_status_sub_ = create_subscription<carla_msgs::msg::CarlaTrafficLightStatusList>(
    tl_status_topic_, rclcpp::QoS(10),
    std::bind(&BehaviorPlannerNode::trafficLightStatusCallback, this, _1));

  tl_info_sub_ = create_subscription<carla_msgs::msg::CarlaTrafficLightInfoList>(
    tl_info_topic_, rclcpp::QoS(10).transient_local().reliable(),
    std::bind(&BehaviorPlannerNode::trafficLightInfoCallback, this, _1));
}

void BehaviorPlannerNode::createPublisher()
{
  cmd_pub_ =
    create_publisher<planning_msgs::msg::BehaviorCommand>(behavior_command_topic_, rclcpp::QoS(10));
}

void BehaviorPlannerNode::registerBTNodes()
{
  registerBTNode<IsAtTJunction>("IsAtTJunction");
  registerBTNode<IsTrafficLightRedOrYellow>("IsTrafficLightRedOrYellow");
  registerBTNode<IsLeadingVehicleDetected>("IsLeadingVehicleDetected");
  registerBTNode<IsLeadingVehicleStatic>("IsLeadingVehicleStatic");
  registerBTNode<IsLeadingVehicleMoving>("IsLeadingVehicleMoving");
  registerBTNode<IsAvoidancePossible>("IsAvoidancePossible");

  registerBTNode<StopAtTrafficLight>("StopAtTrafficLight");
  registerBTNode<RequestAvoidance>("RequestAvoidance");
  registerBTNode<WaitBehindObstacle>("WaitBehindObstacle");
  registerBTNode<FollowLeadingVehicle>("FollowLeadingVehicle");
  registerBTNode<FollowLane>("FollowLane");

  RCLCPP_INFO(get_logger(), "Behavior tree nodes registered");
}

void BehaviorPlannerNode::loadBehaviorTree()
{
  std::string tree_path = behavior_tree_file_;

  // Resolve relative behavior tree path from package share.
  if (tree_path.empty() || tree_path.front() != '/') {
    try {
      const std::string pkg_share =
        ament_index_cpp::get_package_share_directory("behavior_planner");

      tree_path = pkg_share + "/" + tree_path;
    } catch (const std::exception & e) {
      RCLCPP_FATAL(get_logger(), "Failed to resolve behavior tree path: %s", e.what());
      throw;
    }
  }

  try {
    // Load XML after registering all custom BT nodes.
    bt_tree_ = bt_factory_.createTreeFromFile(tree_path);

    RCLCPP_INFO(get_logger(), "Loaded behavior tree: %s", tree_path.c_str());

    // Setup Groot2 publisher if enabled. This allows live visualization of the
    // running tree.
    if (enable_groot_monitoring_) {
      groot_publisher_ = std::make_unique<BT::Groot2Publisher>(bt_tree_);

      RCLCPP_INFO(
        get_logger(),
        "Groot2 live monitoring enabled. Open Groot2 "
        "and connect to the running tree.");
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(
      get_logger(), "Failed to load behavior tree '%s': %s", tree_path.c_str(), e.what());

    throw;
  }
}

void BehaviorPlannerNode::createTickTimer()
{
  const double hz = get_parameter("tick_rate_hz").as_double();

  if (hz <= 0.0) {
    throw std::runtime_error("[BehaviorPlannerNode] tick_rate_hz must be > 0");
  }

  const auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / hz));

  tick_timer_ = create_wall_timer(period, std::bind(&BehaviorPlannerNode::tick, this));
}

void BehaviorPlannerNode::routeCallback(const planning_msgs::msg::Route::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);

  ctx_->global_route = *msg;
  ctx_->route_received = true;
  ctx_->goal_reached = false;

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 3000, "Received route: %zu waypoints, %.2f m",
    ctx_->global_route.waypoints.size(), ctx_->global_route.total_length_m);
}

void BehaviorPlannerNode::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);

  ctx_->ego_odom = *msg;
  ctx_->ego_odom_received = true;
}

void BehaviorPlannerNode::objectsCallback(
  const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);

  // Don't update objects until we have the ego odometry, since we need it to filter and
  if (!ctx_->ego_odom_received) {
    return;
  }

  // Convert raw CARLA objects into simplified planner objects.
  ctx_->objects = parseObjects(msg);
  last_objects_stamp_ = now();
  objects_ever_received_ = true;
}

void BehaviorPlannerNode::trafficLightStatusCallback(
  const carla_msgs::msg::CarlaTrafficLightStatusList::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);

  for (const auto & status : msg->traffic_lights) {
    tl_status_map_[status.id] = status.state;
  }

  fuseTrafficLights();
}

void BehaviorPlannerNode::trafficLightInfoCallback(
  const carla_msgs::msg::CarlaTrafficLightInfoList::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(ctx_->mtx);

  for (const auto & info : msg->traffic_lights) {
    tl_position_map_[info.id] = {info.transform.position.x, info.transform.position.y};
  }

  fuseTrafficLights();
}

std::vector<TrackedObject> BehaviorPlannerNode::parseObjects(
  const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg)
{
  // Called from objectsCallback while holding ctx_->mtx, so ctx_ fields
  // are safe to read without an extra lock.
  const double ego_x = ctx_->ego_odom.pose.pose.position.x;
  const double ego_y = ctx_->ego_odom.pose.pose.position.y;
  const double filter_sq = ctx_->params.ego_filter_radius * ctx_->params.ego_filter_radius;

  std::vector<TrackedObject> objects;
  objects.reserve(msg->objects.size());

  for (const auto & object : msg->objects) {
    const double dx = object.pose.position.x - ego_x;
    const double dy = object.pose.position.y - ego_y;
    if ((dx * dx + dy * dy) < filter_sq) {
      continue;
    }

    TrackedObject tracked;
    tracked.id = object.id;
    tracked.x = object.pose.position.x;
    tracked.y = object.pose.position.y;
    tracked.z = object.pose.position.z;
    tracked.speed = std::hypot(object.twist.linear.x, object.twist.linear.y);
    tracked.label = "vehicle";

    // forward_distance and route_lateral_distance are filled later inside
    // BehaviorContext::updateLeadingVehicle() on each tick.
    tracked.forward_distance = 0.0;
    tracked.route_lateral_distance = 0.0;

    // Parse obstacle half-length from shape dimensions.
    // shape.dimensions = [d0, d1, height] for a BOX.
    // half_length = max(d0, d1) / 2: the longer axis is the obstacle length.
    // This is used by IsAvoidancePossible to compute a geometry-aware latch
    // release threshold instead of relying on a fixed constant.
    if (object.shape.dimensions.size() >= 2) {
      const double d0 = object.shape.dimensions[0];
      const double d1 = object.shape.dimensions[1];
      tracked.half_length = 0.5 * std::max(d0, d1);
    }

    objects.push_back(tracked);
  }

  return objects;
}

void BehaviorPlannerNode::fuseTrafficLights()
{
  ctx_->traffic_lights.clear();
  ctx_->traffic_lights.reserve(tl_status_map_.size());

  for (const auto & [id, state] : tl_status_map_) {
    const auto pos_it = tl_position_map_.find(id);

    // Status without position cannot be used for distance-based behavior.
    if (pos_it == tl_position_map_.end()) {
      continue;
    }

    TrafficLightEntry entry;
    entry.id = id;
    entry.state = state;
    entry.x = pos_it->second.first;
    entry.y = pos_it->second.second;
    entry.z = 0.0;

    ctx_->traffic_lights.push_back(entry);
  }
}

void BehaviorPlannerNode::tick()
{
  // Lock 1: update derived state, then release
  {
    std::lock_guard<std::mutex> lock(ctx_->mtx);

    if (!ctx_->ego_odom_received) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for odometry on '%s'...",
        odometry_topic_.c_str());
      return;
    }
    if (!ctx_->route_received) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for route on '%s': set a goal in RViz2",
        global_route_topic_.c_str());
      return;
    }

    ctx_->updateGoalReached();
    if (ctx_->goal_reached) {
      RCLCPP_INFO_ONCE(get_logger(), "Goal reached: behavior planner idle.");
      return;
    }
    // Clear stale objects if publisher has stopped sending messages.
    // Without this, the last received ObjectArray stays in ctx_->objects forever,
    // causing the vehicle to remain in WAIT_BEHIND_OBSTACLE even after the obstacle
    // disappears (publisher terminated).
    {
      if (objects_ever_received_) {
        const double age = (now() - last_objects_stamp_).seconds();
        if (age > kObjectsTimeoutSec && !ctx_->objects.empty()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "[BehaviorPlanner] Objects stale (%.2f s): clearing obstacle list", age);
          ctx_->objects.clear();
        }
      }
    }

    ctx_->updateLeadingVehicle();
    ctx_->updateRelevantTrafficLight();
    ctx_->updateHysteresis();
  }  // lock released: callbacks unblocked

  // BT walk: no lock
  bt_tree_.tickOnce();  // actions write ctx_->current_command here

  // Lock 2: capture what the BT just wrote, then publish
  planning_msgs::msg::BehaviorCommand cmd;
  {
    std::lock_guard<std::mutex> lock(ctx_->mtx);
    cmd = ctx_->current_command;
  }

  cmd.header.stamp = now();
  cmd.header.frame_id = "map";
  cmd_pub_->publish(cmd);
}

}  // namespace behavior_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<behavior_planner::BehaviorPlannerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("behavior_planner_node"), "Fatal error: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
