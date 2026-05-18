#ifndef VELOCITY_PROFILER__VELOCITY_PROFILER_NODE_HPP_
#define VELOCITY_PROFILER__VELOCITY_PROFILER_NODE_HPP_

#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <planning_msgs/msg/trajectory.hpp>
#include <planning_msgs/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "velocity_profiler/velocity_profiler.hpp"

namespace velocity_profiler
{

/**
 * @brief ROS 2 wrapper node for the pure C++ velocity profiler.
 *
 * Receives a local planner trajectory, generates a velocity profile, and
 * publishes the timed trajectory for the controller.
 */
class VelocityProfilerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the velocity profiler node.
   *
   * @param options ROS 2 node options.
   */
  explicit VelocityProfilerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Load ROS parameters into the profiler configuration.
   */
  void loadParams();

  /**
   * @brief Create subscribers, publishers, and timers.
   */
  void createInterfaces();

  /**
   * @brief Handle incoming local planner trajectory messages.
   *
   * @param msg Input trajectory from the local planner.
   */
  void onTrajectory(const planning_msgs::msg::Trajectory::SharedPtr msg);

  /**
   * @brief Update ego speed from odometry.
   *
   * @param msg Ego vehicle odometry message.
   */
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Periodically publish the latest profiled trajectory and debug data.
   */
  void onPublishTimer();

  /**
   * @brief Convert a ROS trajectory message into profiler waypoints.
   *
   * @param msg Input ROS trajectory.
   * @return Internal waypoint vector.
   */
  std::vector<StampedWaypoint> trajectoryToWaypoints(
    const planning_msgs::msg::Trajectory & msg) const;

  /**
   * @brief Convert profiled waypoints back to a ROS trajectory message.
   *
   * @param wps Profiled waypoints.
   * @param header Header to preserve timestamp and frame.
   * @return ROS trajectory message.
   */
  planning_msgs::msg::Trajectory waypointsToTrajectory(
    const std::vector<StampedWaypoint> & wps, const std_msgs::msg::Header & header) const;

  /**
   * @brief Build RViz markers for the profiled trajectory.
   *
   * @param traj Profiled trajectory.
   * @return Marker array for visualization.
   */
  visualization_msgs::msg::MarkerArray buildDebugMarkers(
    const planning_msgs::msg::Trajectory & traj) const;

  /**
   * @brief Build debug speed array from the profiled trajectory.
   *
   * @param traj Profiled trajectory.
   * @return Array containing waypoint speeds.
   */
  std_msgs::msg::Float32MultiArray buildSpeedArray(
    const planning_msgs::msg::Trajectory & traj) const;

  rclcpp::Subscription<planning_msgs::msg::Trajectory>::SharedPtr
    sub_trajectory_;  ///< Local planner trajectory subscriber.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    sub_odometry_;  ///< Ego odometry subscriber.

  rclcpp::Publisher<planning_msgs::msg::Trajectory>::SharedPtr
    pub_trajectory_;  ///< Profiled trajectory publisher.
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr
    pub_speed_profile_;  ///< Debug speed profile publisher.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    pub_markers_;  ///< RViz marker publisher.

  rclcpp::TimerBase::SharedPtr publish_timer_;  ///< Timer for periodic output publishing.

  VelocityProfiler profiler_;  ///< Pure C++ velocity profiler.
  ProfilerParams params_;      ///< Active profiler parameters.

  double ego_speed_{0.0};  ///< Latest ego speed from odometry [m/s].

  std::optional<planning_msgs::msg::Trajectory>
    current_trajectory_;  ///< Latest profiled trajectory.
  rclcpp::Time last_trajectory_update_;  ///< Wall-clock receipt time of current_trajectory_.
  double trajectory_timeout_{0.5};  ///< Max age [s] before current_trajectory_ is dropped.

  std::string map_frame_{"map"};  ///< Frame used for visualization markers.
};

}  // namespace velocity_profiler

#endif  // VELOCITY_PROFILER__VELOCITY_PROFILER_NODE_HPP_
