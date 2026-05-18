#ifndef CONTROLLER__CONTROLLER_NODE_HPP_
#define CONTROLLER__CONTROLLER_NODE_HPP_

#include <carla_msgs/msg/carla_ego_vehicle_control.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <planning_msgs/msg/trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "controller/pure_pursuit_controller.hpp"
#include "controller/vehicle_model.hpp"

namespace controller
{

/**
 * @class ControllerNode
 * @brief ROS 2 node for trajectory tracking control.
 *
 * ControllerNode subscribes to the velocity-profiled trajectory and ego odometry,
 * converts them into internal controller data, runs Pure Pursuit lateral control
 * with longitudinal speed tracking, and publishes CARLA ego vehicle control
 * commands.
 *
 * Responsibilities:
 * - receive planned trajectory from the velocity profiler
 * - receive ego vehicle odometry from CARLA
 * - validate data freshness
 * - convert ROS messages to internal controller structures
 * - compute throttle, brake, and steering commands
 * - publish safe zero/brake commands when data is invalid
 */
class ControllerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the controller node.
   *
   * Initializes parameters, controller/model objects, ROS interfaces, and the
   * periodic control timer.
   *
   * @param options ROS 2 node options.
   */
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  /**
   * @brief Declare all ROS parameters used by the controller.
   */
  void declareParameters();

  /**
   * @brief Load ROS parameters into controller, vehicle model, and node settings.
   */
  void loadParameters();

  /**
   * @brief Validate loaded parameters and throw if any required value is invalid.
   */
  void validateParameters() const;

  /**
   * @brief Create ROS subscriptions, publisher, and timer.
   */
  void createInterfaces();

  /**
   * @brief Store the latest trajectory message from the velocity profiler.
   *
   * @param msg Velocity-profiled trajectory message.
   */
  void onTrajectory(const planning_msgs::msg::Trajectory::ConstSharedPtr & msg);

  /**
   * @brief Store the latest ego odometry message from CARLA.
   *
   * @param msg Ego vehicle odometry message.
   */
  void onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  /**
   * @brief Periodic control loop callback.
   */
  void onControlTimer();

  /**
   * @brief Convert planning_msgs::msg::Trajectory to internal trajectory points.
   *
   * Reads:
   * - pose x/y/yaw for Pure Pursuit geometry
   * - longitudinal_velocity_mps for target speed tracking
   * - is_stop_point for hard stop / brake override
   *
   * This function intentionally does not use max_velocity_mps because that is a
   * planning constraint from upstream modules, while longitudinal_velocity_mps is
   * the final velocity-profiled command.
   *
   * @param traj Input trajectory message.
   * @param state Current ego vehicle state.
   * @return Internal trajectory points used by the controller.
   */
  std::vector<TrajectoryPoint> trajectoryToPoints(
    const planning_msgs::msg::Trajectory & traj, const VehicleState & state) const;

  /**
   * @brief Find the nearest trajectory waypoint to the ego vehicle.
   *
   * @param traj Input trajectory message.
   * @param state Current ego vehicle state.
   * @param start_from Index to start searching from.
   * @return Index of the nearest trajectory point.
   */
  std::size_t findNearestIndex(
    const planning_msgs::msg::Trajectory & traj, const VehicleState & state,
    std::size_t start_from) const;

  /**
   * @brief Convert odometry message into internal vehicle state.
   *
   * @param odom Ego odometry message.
   * @return Internal vehicle state used by the controller.
   */
  VehicleState odomToState(const nav_msgs::msg::Odometry & odom) const;

  /**
   * @brief Publish a safe zero-throttle control command.
   *
   * Used when input data is missing, stale, invalid, or when the controller must
   * force the vehicle to stop.
   *
   * @param brake Brake command value to publish.
   */
  void publishZero(double brake = 0.3);

  /**
   * @brief Check whether the latest trajectory and odometry are available and fresh.
   *
   * @return true if the controller has valid input data, otherwise false.
   */
  bool dataValid() const;

  /**
   * @brief Get the configured control period.
   *
   * @return Control period in seconds.
   */
  double controlPeriod() const;

  rclcpp::Subscription<planning_msgs::msg::Trajectory>::SharedPtr
    trajectory_sub_;  ///< Subscription to the velocity-profiled trajectory.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odometry_sub_;  ///< Subscription to ego vehicle odometry.
  rclcpp::Publisher<carla_msgs::msg::CarlaEgoVehicleControl>::SharedPtr
    control_pub_;  ///< Publisher for CARLA ego vehicle control commands.
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< Timer that runs the controller loop.

  std::optional<planning_msgs::msg::Trajectory>
    latest_trajectory_;  ///< Latest received trajectory, if available.
  std::optional<nav_msgs::msg::Odometry>
    latest_odometry_;  ///< Latest received ego odometry, if available.
  rclcpp::Time trajectory_stamp_{
    0, 0, RCL_ROS_TIME};  ///< Timestamp of the latest trajectory message.
  rclcpp::Time odometry_stamp_{0, 0, RCL_ROS_TIME};  ///< Timestamp of the latest odometry message.

  double previous_front_steer_{
    0.0};  ///< Previous front steering command used for steering-rate limiting.
  double filtered_steer_{0.0};  ///< Filtered steering command.
  double filtered_accel_{0.0};  ///< Filtered longitudinal acceleration command.

  std::unique_ptr<PurePursuitController> controller_;  ///< Pure Pursuit controller implementation.
  std::unique_ptr<VehicleModel>
    vehicle_model_;  ///< Vehicle model used for command conversion and limits.

  VehicleModelParams model_params_;  ///< Vehicle model configuration parameters.
  PurePursuitControllerParams
    controller_params_;  ///< Pure Pursuit controller configuration parameters.

  std::string trajectory_topic_;  ///< Input trajectory topic name.
  std::string odometry_topic_;    ///< Input odometry topic name.
  std::string control_topic_;     ///< Output CARLA control command topic name.

  double publish_rate_{20.0};       ///< Control loop publishing rate in Hz.
  double trajectory_timeout_{1.0};  ///< Maximum allowed age of the latest trajectory in seconds.
  double odometry_timeout_{0.5};    ///< Maximum allowed age of the latest odometry in seconds.
  int min_trajectory_points_{
    5};  ///< Minimum number of trajectory points required to run the controller.
  int reference_points_{
    30};  ///< Number of trajectory points used as the controller reference horizon.
};

}  // namespace controller

#endif  // CONTROLLER__CONTROLLER_NODE_HPP_
