#ifndef LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
#define LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_

#include <cstddef>
#include <derived_object_msgs/msg/object_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <planning_msgs/msg/behavior_command.hpp>
#include <planning_msgs/msg/route.hpp>
#include <planning_msgs/msg/trajectory.hpp>
#include <planning_msgs/msg/trajectory_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "local_planner/avoidance_latch.hpp"
#include "local_planner/hybrid_astar.hpp"
#include "local_planner/obstacle_processor.hpp"
#include "local_planner/stop_latch.hpp"

namespace local_planner
{

/**
 * @class LocalPlannerNode
 * @brief ROS 2 node responsible for local trajectory generation.
 *
 * LocalPlannerNode consumes the global route, ego odometry, perceived objects,
 * and behavior commands.  It produces a local trajectory for the velocity
 * profiler and a debug path for RViz visualization.
 *
 * ## Internal structure
 * Logic is split across three helper classes to keep this node thin:
 * - ObstacleProcessor: converts raw object messages into Obstacle circles.
 * - AvoidanceLatch   : manages the committed Hybrid A* path state machine.
 * - StopLatch        : manages the committed stop-point state machine.
 *
 * The node itself handles: ROS I/O, parameter loading, the main planning
 * dispatch (computeTrajectory), route-following path building, and message
 * serialization.
 */
class LocalPlannerNode : public rclcpp::Node
{
public:
  explicit LocalPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  /// Fallback speed used before the first BehaviorCommand is received [m/s].
  static constexpr double kFallbackSpeed = 8.0;

  // Setup
  void declareParameters();
  void loadParams();
  void createInterfaces();

  // ROS callbacks
  void onRoute(const planning_msgs::msg::Route::ConstSharedPtr & msg);
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void onObjects(const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg);
  void onBehaviorCommand(const planning_msgs::msg::BehaviorCommand::ConstSharedPtr & msg);
  void onPublishTimer();

  // Planning dispatch

  /**
   * @brief Top-level planning function called every timer tick.
   *
   * Checks committed stop, goal reached, avoidance, or normal lane following
   * in that order, and writes the result into current_trajectory_.
   */
  void computeTrajectory();

  /**
   * @brief Return true if the current behavior is a stopping behavior.
   */
  bool isStoppingBehavior() const;

  /**
   * @brief Return true if AVOID_STATIC_OBSTACLE is the current behavior.
   */
  bool isAvoidanceRequested() const;

  /**
   * @brief Handle the goal-reached case.
   * @return True if goal was reached (caller should return early).
   */
  bool handleGoalReached(const Pose & ego);

  /**
   * @brief Run Hybrid A* and update the avoidance latch.
   * @return True if an avoidance trajectory was produced (or latched path followed).
   */
  bool computeAvoidanceTrajectory(const Pose & ego, std::size_t closest_idx);

  // Route-following path building

  std::vector<TrajectoryPoint> buildInterpolatedPath(std::size_t closest_idx) const;
  double getPlanningHorizon(bool is_stopping) const;
  std::vector<TrajectoryPoint> buildRouteFollowingPath(
    std::size_t closest_idx, double planning_horizon) const;
  void applyStopConstraint(std::vector<TrajectoryPoint> & path, bool is_traffic_light_stop) const;
  std::vector<TrajectoryPoint> smoothRouteFollowingPath(
    std::vector<TrajectoryPoint> path, bool is_stopping, bool is_traffic_light_stop) const;

  // Utility

  Pose getCurrentEgoPose() const;
  std::size_t findClosestWaypoint(double ego_x, double ego_y) const;
  RouteCorridor buildRouteCorridor(std::size_t start_idx, std::size_t end_idx) const;
  TrajectoryPoint makeStopPoint(const Pose & pose, const std::string & reason) const;
  void publishFallbackStopTrajectory(const Pose & ego, const std::string & reason);

  planning_msgs::msg::Trajectory toTrajectory(
    const std::vector<TrajectoryPoint> & points, const rclcpp::Time & stamp) const;
  nav_msgs::msg::Path toNavPath(
    const std::vector<TrajectoryPoint> & points, const rclcpp::Time & stamp) const;

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion & q);

  // Planning components
  std::unique_ptr<HybridAStar> planner_{nullptr};
  std::unique_ptr<ObstacleProcessor> obstacle_processor_{nullptr};
  std::unique_ptr<AvoidanceLatch> avoidance_latch_{nullptr};
  std::unique_ptr<StopLatch> stop_latch_{nullptr};

  // ROS interfaces
  rclcpp::Subscription<planning_msgs::msg::Route>::SharedPtr route_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr objects_sub_;
  rclcpp::Subscription<planning_msgs::msg::BehaviorCommand>::SharedPtr behavior_cmd_sub_;
  rclcpp::Publisher<planning_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Shared state
  planning_msgs::msg::Route::ConstSharedPtr current_route_{nullptr};
  nav_msgs::msg::Odometry::ConstSharedPtr last_odom_{nullptr};
  std::vector<TrajectoryPoint> current_trajectory_;
  std::size_t last_closest_idx_{0};

  planning_msgs::msg::BehaviorCommand current_behavior_{};
  bool behavior_received_{false};
  std::string behavior_command_topic_;
  double current_speed_limit_{kFallbackSpeed};

  // Parameters
  VehicleParams vehicle_params_;
  HybridAStarParams hybrid_params_;

  bool enable_path_smoothing_{true};
  double lookahead_distance_{30.0};
  double min_waypoint_spacing_{0.5};
  std::size_t output_points_{100};

  std::size_t search_window_size_{100};
  std::size_t look_behind_points_{2};
  double max_cross_track_error_{3.0};
  double goal_reached_threshold_{1.5};

  double stop_creep_speed_{0.5};
  double stop_creep_distance_{5.0};
  double stop_commit_distance_{4.0};

  // Obstacle / corridor params (forwarded to ObstacleProcessor).
  double obstacle_lookahead_{30.0};
  double obstacle_inflation_{0.8};
  double corridor_width_multiplier_{1.5};
  double avoidance_corridor_multiplier_{3.0};
  double ego_filter_multiplier_{0.5};
  double road_boundary_margin_{0.5};
  double min_avoidance_corridor_half_width_{5.2};
  double avoidance_release_distance_{6.0};

  std::string frame_id_{"map"};
  std::string trajectory_topic_;
  std::string debug_topic_;
};

}  // namespace local_planner

#endif  // LOCAL_PLANNER__LOCAL_PLANNER_NODE_HPP_
