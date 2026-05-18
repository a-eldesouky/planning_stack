#include "velocity_profiler/velocity_profiler_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

namespace velocity_profiler
{

VelocityProfilerNode::VelocityProfilerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("velocity_profiler_node", options), profiler_(ProfilerParams{}),
  last_trajectory_update_(now())
{
  RCLCPP_INFO(get_logger(), "=== Velocity Profiler Node Starting ===");

  loadParams();
  profiler_.setParams(params_);
  createInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "Velocity Profiler ready | cruise=%.1f km/h | a_max=%.1f m/s² | "
    "a_dec=%.1f m/s² | j_max=%.1f m/s³",
    params_.cruise_speed * 3.6, params_.a_max, params_.a_dec, params_.j_max);
  RCLCPP_INFO(
    get_logger(),
    "Subscriptions: /local_planner/trajectory + /carla/ego_vehicle/odometry only."
    " No BehaviorStatus subscription. Constraints come from the trajectory.");
}

void VelocityProfilerNode::loadParams()
{
  params_.cruise_speed = declare_parameter<double>("cruise_speed", 8.33);
  params_.max_speed = declare_parameter<double>("max_speed", 13.89);
  params_.a_max = declare_parameter<double>("a_max", 2.0);
  params_.a_dec = declare_parameter<double>("a_dec", 3.0);
  params_.j_max = declare_parameter<double>("j_max", 1.5);
  params_.a_lat_max = declare_parameter<double>("a_lat_max", 2.5);
  params_.min_curvature_threshold = declare_parameter<double>("min_curvature_threshold", 1e-4);
  params_.min_ds = declare_parameter<double>("min_ds", 0.01);

  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  trajectory_timeout_ = declare_parameter<double>("trajectory_timeout", 0.5);

  // ego_speed is the only runtime context needed from outside
  ego_speed_ = 0.0;
}

void VelocityProfilerNode::createInterfaces()
{
  //  Subscriptions
  sub_trajectory_ = create_subscription<planning_msgs::msg::Trajectory>(
    "/local_planner/trajectory", rclcpp::QoS(10),
    [this](const planning_msgs::msg::Trajectory::SharedPtr msg) { onTrajectory(msg); });

  sub_odometry_ = create_subscription<nav_msgs::msg::Odometry>(
    "/carla/ego_vehicle/odometry", rclcpp::QoS(10),
    [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdometry(msg); });

  //  Publications
  pub_trajectory_ = create_publisher<planning_msgs::msg::Trajectory>(
    "/velocity_profiler/trajectory", rclcpp::QoS(10));

  pub_speed_profile_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/velocity_profiler/debug/speed_profile", rclcpp::QoS(10));

  pub_markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/velocity_profiler/debug/markers", rclcpp::QoS(10));

  publish_timer_ = create_wall_timer(100ms, [this] { onPublishTimer(); });
}

void VelocityProfilerNode::onTrajectory(const planning_msgs::msg::Trajectory::SharedPtr msg)
{
  if (msg->points.size() < 2) {
    if (msg->points.size() == 1 && msg->points[0].is_stop_point) {
      planning_msgs::msg::Trajectory out;
      out.header = msg->header;
      auto pt = msg->points[0];
      pt.longitudinal_velocity_mps = 0.0f;
      pt.acceleration_mps2 = 0.0f;
      pt.time_from_start.sec = 0;
      pt.time_from_start.nanosec = 0;
      out.points.push_back(pt);
      current_trajectory_ = out;
      last_trajectory_update_ = now();
      RCLCPP_DEBUG(get_logger(), "Single-point stop trajectory forwarded.");
      return;
    }

    RCLCPP_WARN(
      get_logger(), "Received trajectory with %zu points (< 2): skipping.", msg->points.size());
    return;
  }

  // Convert to internal waypoints
  auto wps = trajectoryToWaypoints(*msg);
  if (wps.empty()) {
    return;
  }

  // Run three-pass profiler: only ego_speed needed as external context
  try {
    const auto result = profiler_.generate(std::move(wps), ego_speed_);
    current_trajectory_ = waypointsToTrajectory(result, msg->header);
    last_trajectory_update_ = now();
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "VelocityProfiler::generate() failed: %s", ex.what());
    // Drop the stale trajectory rather than keep republishing it as if it were fresh.
    current_trajectory_.reset();
  }
}

void VelocityProfilerNode::onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const auto & twist = msg->twist.twist.linear;
  ego_speed_ = std::hypot(twist.x, twist.y);
}

void VelocityProfilerNode::onPublishTimer()
{
  if (!current_trajectory_.has_value()) {
    return;
  }

  if ((now() - last_trajectory_update_).seconds() > trajectory_timeout_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "No trajectory update in over %.2f s: dropping the stale profile.", trajectory_timeout_);
    current_trajectory_.reset();
    return;
  }

  auto & traj = *current_trajectory_;

  traj.header.stamp = now();

  pub_trajectory_->publish(traj);

  if (pub_speed_profile_->get_subscription_count() > 0) {
    pub_speed_profile_->publish(buildSpeedArray(traj));
  }

  if (pub_markers_->get_subscription_count() > 0) {
    pub_markers_->publish(buildDebugMarkers(traj));
  }
}

std::vector<StampedWaypoint> VelocityProfilerNode::trajectoryToWaypoints(
  const planning_msgs::msg::Trajectory & msg) const
{
  std::vector<StampedWaypoint> wps;
  wps.reserve(msg.points.size());

  for (const auto & tp : msg.points) {
    StampedWaypoint wp;
    wp.x = tp.pose.position.x;
    wp.y = tp.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(tp.pose.orientation, q);
    double roll{0}, pitch{0}, yaw{0};
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    wp.yaw = yaw;

    // Copy local-planner constraints verbatim
    wp.max_velocity_mps = tp.max_velocity_mps;
    wp.is_stop_point = tp.is_stop_point;
    wp.stop_reason = tp.stop_reason;

    wps.push_back(wp);
  }

  return wps;
}

planning_msgs::msg::Trajectory VelocityProfilerNode::waypointsToTrajectory(
  const std::vector<StampedWaypoint> & wps, const std_msgs::msg::Header & header) const
{
  planning_msgs::msg::Trajectory traj;
  traj.header = header;
  traj.points.reserve(wps.size());

  for (const auto & wp : wps) {
    planning_msgs::msg::TrajectoryPoint tp;

    tp.pose.position.x = wp.x;
    tp.pose.position.y = wp.y;
    tp.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, wp.yaw);
    tp.pose.orientation = tf2::toMsg(q);

    // Local planner constraints: preserved unchanged
    tp.max_velocity_mps = wp.max_velocity_mps;
    tp.is_stop_point = wp.is_stop_point;
    tp.stop_reason = wp.stop_reason;

    tp.longitudinal_velocity_mps = static_cast<float>(wp.velocity);
    tp.acceleration_mps2 = static_cast<float>(wp.acceleration);
    tp.time_from_start.sec = static_cast<int32_t>(wp.time_from_start);
    tp.time_from_start.nanosec =
      static_cast<uint32_t>((wp.time_from_start - std::floor(wp.time_from_start)) * 1e9);

    traj.points.push_back(tp);
  }

  return traj;
}

visualization_msgs::msg::MarkerArray VelocityProfilerNode::buildDebugMarkers(
  const planning_msgs::msg::Trajectory & traj) const
{
  visualization_msgs::msg::MarkerArray arr;

  visualization_msgs::msg::Marker clear;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  arr.markers.push_back(clear);

  const double v_max = params_.cruise_speed;

  for (std::size_t i = 0; i < traj.points.size(); ++i) {
    const auto & pt = traj.points[i];

    visualization_msgs::msg::Marker m;
    m.header = traj.header;
    m.ns = "velocity_profile";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;

    m.pose.position.x = pt.pose.position.x;
    m.pose.position.y = pt.pose.position.y;
    m.pose.position.z = pt.is_stop_point ? 0.4 : 0.1;  // elevate stop points
    m.pose.orientation.w = 1.0;

    m.scale.x = pt.is_stop_point ? 0.6 : 0.3;
    m.scale.y = m.scale.x;
    m.scale.z = m.scale.x;

    const double ratio =
      std::clamp(static_cast<double>(pt.longitudinal_velocity_mps) / v_max, 0.0, 1.0);
    m.color.a = 1.0f;
    m.color.r = pt.is_stop_point ? 1.0f : static_cast<float>(ratio);
    m.color.g = pt.is_stop_point ? 0.0f : static_cast<float>(1.0 - std::abs(2.0 * ratio - 1.0));
    m.color.b = pt.is_stop_point ? 0.0f : static_cast<float>(1.0 - ratio);

    m.lifetime = rclcpp::Duration(0, 150'000'000);
    arr.markers.push_back(m);
  }

  return arr;
}

std_msgs::msg::Float32MultiArray VelocityProfilerNode::buildSpeedArray(
  const planning_msgs::msg::Trajectory & traj) const
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.reserve(traj.points.size());
  for (const auto & pt : traj.points) {
    msg.data.push_back(pt.longitudinal_velocity_mps);
  }
  return msg;
}

}  // namespace velocity_profiler

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<velocity_profiler::VelocityProfilerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("velocity_profiler_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
