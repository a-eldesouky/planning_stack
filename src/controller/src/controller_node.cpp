#include "controller/controller_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <limits>
#include <stdexcept>

namespace controller
{
namespace
{

// Converts a ROS pose quaternion into planar yaw.
// The controller only needs the vehicle heading in the map frame, so roll/pitch are ignored.
double yawFromPose(const geometry_msgs::msg::Pose & pose)
{
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  double roll{0.0}, pitch{0.0}, yaw{0.0};
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  return yaw;
}

}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  declareParameters();
  loadParameters();
  validateParameters();

  // VehicleModel owns actuator limits and CARLA conversion; PurePursuitController owns tracking logic.
  // This keeps the ROS node mostly as an integration layer.
  vehicle_model_ = std::make_unique<VehicleModel>(model_params_);
  controller_ = std::make_unique<PurePursuitController>(controller_params_, model_params_);

  createInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "Pure Pursuit controller started | rate=%.1f Hz | fallback_speed=%.2f m/s"
    " | input=/velocity_profiler/trajectory",
    publish_rate_, controller_params_.reference_speed);
}

void ControllerNode::declareParameters()
{
  // Topic parameters make the node reusable with simulation, rosbag replay, or a different planning stack.
  declare_parameter("topics.trajectory", "/velocity_profiler/trajectory");
  declare_parameter("topics.odometry", "/carla/ego_vehicle/odometry");
  declare_parameter("topics.control_cmd", "/carla/ego_vehicle/vehicle_control_cmd");

  // Vehicle geometry and actuator limits. These parameters should match the CARLA ego vehicle model.
  declare_parameter("vehicle.wheelbase", 1.545);
  declare_parameter("vehicle.lf", 0.7725);
  declare_parameter("vehicle.lr", 0.7725);
  declare_parameter("vehicle.max_front_steer", 0.2792);
  declare_parameter("vehicle.max_rear_steer", 0.2792);
  declare_parameter("vehicle.max_steering_rate", 0.17);
  declare_parameter("vehicle.max_accel", 3.0);
  declare_parameter("vehicle.max_decel", 4.0);
  declare_parameter("vehicle.max_throttle", 1.0);
  declare_parameter("vehicle.max_brake", 1.0);

  // Rear steering exists only in the kinematic model; CARLA cannot actuate it (see validateParameters).
  declare_parameter("double_ackermann.enable_rear_steering", false);
  declare_parameter("double_ackermann.rear_steering_ratio", 0.3);

  // Pure Pursuit and longitudinal control tuning.
  declare_parameter("controller.reference_speed", 5.0);
  declare_parameter("controller.lookahead_distance", 6.0);
  declare_parameter("controller.min_lookahead_distance", 4.0);
  declare_parameter("controller.lookahead_time_gain", 0.3);
  declare_parameter("controller.kp_speed", 0.8);
  declare_parameter("controller.ki_speed", 0.05);
  declare_parameter("controller.kd_speed", 0.02);
  declare_parameter("controller.integral_limit", 35.0);
  declare_parameter("controller.steer_filter_alpha", 0.7);
  declare_parameter("controller.accel_filter_alpha", 0.8);

  // Runtime freshness limits. If trajectory or odometry becomes stale, the controller publishes zero.
  declare_parameter("control.publish_rate", 20.0);
  declare_parameter("control.trajectory_timeout", 1.0);
  declare_parameter("control.odometry_timeout", 0.5);

  // Number of forward trajectory points used by the controller after trimming near the ego pose.
  declare_parameter("trajectory.reference_points", 30);
  declare_parameter("trajectory.min_points", 2);
}

void ControllerNode::loadParameters()
{
  // Read ROS parameters once during startup. Dynamic reconfigure is intentionally not handled here.
  trajectory_topic_ = get_parameter("topics.trajectory").as_string();
  odometry_topic_ = get_parameter("topics.odometry").as_string();
  control_topic_ = get_parameter("topics.control_cmd").as_string();

  model_params_.wheelbase = get_parameter("vehicle.wheelbase").as_double();
  model_params_.lf = get_parameter("vehicle.lf").as_double();
  model_params_.lr = get_parameter("vehicle.lr").as_double();
  model_params_.max_front_steer = get_parameter("vehicle.max_front_steer").as_double();
  model_params_.max_rear_steer = get_parameter("vehicle.max_rear_steer").as_double();
  model_params_.max_steering_rate = get_parameter("vehicle.max_steering_rate").as_double();
  model_params_.max_accel = get_parameter("vehicle.max_accel").as_double();
  model_params_.max_decel = get_parameter("vehicle.max_decel").as_double();
  model_params_.max_throttle = get_parameter("vehicle.max_throttle").as_double();
  model_params_.max_brake = get_parameter("vehicle.max_brake").as_double();
  model_params_.enable_rear_steering =
    get_parameter("double_ackermann.enable_rear_steering").as_bool();
  model_params_.rear_steering_ratio =
    get_parameter("double_ackermann.rear_steering_ratio").as_double();

  controller_params_.reference_speed = get_parameter("controller.reference_speed").as_double();
  controller_params_.lookahead_distance =
    get_parameter("controller.lookahead_distance").as_double();
  controller_params_.min_lookahead_distance =
    get_parameter("controller.min_lookahead_distance").as_double();
  controller_params_.lookahead_time_gain =
    get_parameter("controller.lookahead_time_gain").as_double();
  controller_params_.kp_speed = get_parameter("controller.kp_speed").as_double();
  controller_params_.ki_speed = get_parameter("controller.ki_speed").as_double();
  controller_params_.kd_speed = get_parameter("controller.kd_speed").as_double();
  controller_params_.integral_limit = get_parameter("controller.integral_limit").as_double();
  controller_params_.steer_filter_alpha =
    get_parameter("controller.steer_filter_alpha").as_double();
  controller_params_.accel_filter_alpha =
    get_parameter("controller.accel_filter_alpha").as_double();

  publish_rate_ = get_parameter("control.publish_rate").as_double();
  trajectory_timeout_ = get_parameter("control.trajectory_timeout").as_double();
  odometry_timeout_ = get_parameter("control.odometry_timeout").as_double();
  reference_points_ = get_parameter("trajectory.reference_points").as_int();
  min_trajectory_points_ = get_parameter("trajectory.min_points").as_int();
}

void ControllerNode::validateParameters() const
{
  // Fail fast during startup instead of letting invalid parameters create unsafe control commands.
  if (publish_rate_ <= 0.0) {
    throw std::invalid_argument("control.publish_rate must be positive");
  }
  if (reference_points_ < 2) {
    throw std::invalid_argument("trajectory.reference_points must be at least 2");
  }
  if (min_trajectory_points_ < 2) {
    throw std::invalid_argument("trajectory.min_points must be at least 2");
  }
  if (model_params_.max_front_steer <= 0.0) {
    throw std::invalid_argument("vehicle.max_front_steer must be positive");
  }
  // With rear steering on, Pure Pursuit solves for a smaller front angle that assumes the
  // rear axle also steers. CARLA's vehicle control has a single steer field, so the rear
  // command would be dropped and the car would under-steer. Refuse instead.
  if (model_params_.enable_rear_steering) {
    throw std::invalid_argument(
            "double_ackermann.enable_rear_steering=true is not supported: "
            "CARLA accepts only a front steering command");
  }
}

void ControllerNode::createInterfaces()
{
  // Trajectory and command topics are part of the planning/control pipeline, so reliable QoS is preferred.
  const auto reliable_qos = rclcpp::QoS(10).reliable();

  // Odometry is high-rate sensor/state data. SensorDataQoS avoids blocking the controller on old samples.
  const auto sensor_qos = rclcpp::SensorDataQoS();

  trajectory_sub_ = create_subscription<planning_msgs::msg::Trajectory>(
    trajectory_topic_, reliable_qos,
    [this](const planning_msgs::msg::Trajectory::ConstSharedPtr & msg) { onTrajectory(msg); });

  odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odometry_topic_, sensor_qos,
    [this](const nav_msgs::msg::Odometry::ConstSharedPtr & msg) { onOdometry(msg); });

  control_pub_ =
    create_publisher<carla_msgs::msg::CarlaEgoVehicleControl>(control_topic_, reliable_qos);

  // Fixed-rate timer decouples control output from input callback frequency.
  const auto period = std::chrono::duration<double>(controlPeriod());
  control_timer_ = create_wall_timer(period, [this]() { onControlTimer(); });

  RCLCPP_INFO(get_logger(), "Subscribing trajectory: %s", trajectory_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Subscribing odometry  : %s", odometry_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing control    : %s", control_topic_.c_str());
}

void ControllerNode::onTrajectory(const planning_msgs::msg::Trajectory::ConstSharedPtr & msg)
{
  // Empty trajectories are ignored to avoid replacing a valid previous trajectory with unusable data.
  if (msg->points.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty trajectory: ignoring");
    return;
  }

  // Copy the latest trajectory so the timer callback can consume a stable snapshot.
  latest_trajectory_ = *msg;

  // Use reception time for freshness because upstream message stamps may come from simulation time or bags.
  trajectory_stamp_ = now();
}

void ControllerNode::onOdometry(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // Store the most recent ego state. The control timer decides whether the sample is fresh enough.
  latest_odometry_ = *msg;
  odometry_stamp_ = now();
}

void ControllerNode::onControlTimer()
{
  // Safety fallback: never compute commands from missing or stale trajectory/odometry.
  if (!dataValid()) {
    publishZero();
    controller_->reset();
    return;
  }

  const VehicleState state = odomToState(*latest_odometry_);

  // Trim the incoming trajectory around the ego vehicle and convert message points to controller points.
  const auto trajectory = trajectoryToPoints(*latest_trajectory_, state);

  if (static_cast<int>(trajectory.size()) < min_trajectory_points_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "Reference trajectory too short after trimming: %zu points",
      trajectory.size());
    publishZero();
    controller_->reset();
    return;
  }

  bool should_hard_stop = false;
  std::string stop_reason = "unknown";
  double stop_distance = -1.0;

  // Respect stop points produced by upstream planners/profilers.
  // The controller does not decide why to stop; it only enforces the stop when the point is close enough.
  for (const auto & pt : trajectory) {
    if (pt.is_stop_point && pt.v <= 0.05) {
      const double dx = pt.x - state.x;
      const double dy = pt.y - state.y;
      const double dist_to_stop = std::hypot(dx, dy);

      if (stop_distance < 0.0 || dist_to_stop < stop_distance) {
        stop_distance = dist_to_stop;
        stop_reason = pt.stop_reason;
      }

      // Close-range guard to prevent creeping through a red light, obstacle stop, or final goal.
      if (dist_to_stop <= 1.5) {
        should_hard_stop = true;
        break;
      }
    }
  }

  if (should_hard_stop) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Stop point reached (reason: %s, dist=%.2f m): braking to zero", stop_reason.c_str(),
      stop_distance);
    publishZero(1.0);
    controller_->reset();
    return;
  }

  // Main control computation: lateral tracking through Pure Pursuit plus longitudinal acceleration request.
  const auto output = controller_->computeCommand(state, trajectory, controlPeriod());
  if (!output.valid) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Controller output invalid");
    publishZero();
    controller_->reset();
    return;
  }

  // Apply physical steering-rate limits before converting the command to CARLA actuator space.
  const double limited_steer = vehicle_model_->applySteeringRateLimit(
    output.delta_front, previous_front_steer_, controlPeriod());

  const auto raw_input = vehicle_model_->coordinateSteering(limited_steer, output.acceleration);
  const auto input = vehicle_model_->clampInput(raw_input);
  const auto carla_cmd = vehicle_model_->toCarlaControl(input);

  // Initialize filters from the first computed command to avoid a startup ramp from zero.
  if (filtered_steer_ == 0.0 && filtered_accel_ == 0.0) {
    filtered_steer_ = carla_cmd.steer;
    filtered_accel_ = input.acceleration;
  }

  // Low-pass filtering reduces oscillation in throttle/brake/steering commands.
  filtered_steer_ = controller_params_.steer_filter_alpha * carla_cmd.steer +
                    (1.0 - controller_params_.steer_filter_alpha) * filtered_steer_;
  filtered_accel_ = controller_params_.accel_filter_alpha * input.acceleration +
                    (1.0 - controller_params_.accel_filter_alpha) * filtered_accel_;

  // Re-run acceleration through the vehicle model after filtering to keep throttle/brake within limits.
  const auto filtered_input =
    vehicle_model_->clampInput(ControlInput{input.delta_front, input.delta_rear, filtered_accel_});
  const auto filtered_cmd = vehicle_model_->toCarlaControl(filtered_input);

  // CARLA steering sign adaptation to match ROS conventions.
  const double raw_steer = std::clamp(filtered_steer_, -1.0, 1.0);
  const double inverted_steer = -raw_steer;

  carla_msgs::msg::CarlaEgoVehicleControl command;
  command.header.stamp = now();
  command.header.frame_id = latest_odometry_->header.frame_id;
  command.steer = static_cast<float>(inverted_steer);

  // Use filtered_accel_ sign directly to arbitrate throttle vs brake.
  // This avoids small numerical brake values fighting throttle near zero acceleration.
  command.throttle = static_cast<float>(filtered_accel_ >= 0.0 ? filtered_cmd.throttle : 0.0);
  command.brake = static_cast<float>(filtered_accel_ < 0.0 ? filtered_cmd.brake : 0.0);
  command.hand_brake = false;
  command.reverse = false;
  command.gear = 1;
  control_pub_->publish(command);

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), 1000,
    "[Cmd] throttle=%.3f brake=%.3f steer=%.3f accel=%.3f target_v=%.2f current_v=%.2f",
    command.throttle, command.brake, command.steer, filtered_accel_, output.target_speed, state.v);

  // Store the limited steering angle for the next rate-limit calculation.
  previous_front_steer_ = limited_steer;
}

std::vector<TrajectoryPoint> ControllerNode::trajectoryToPoints(
  const planning_msgs::msg::Trajectory & traj, const VehicleState & state) const
{
  // Start from the closest path point to avoid tracking already-passed trajectory segments.
  const std::size_t start = findNearestIndex(traj, state, 0);
  const std::size_t end =
    std::min(traj.points.size(), start + static_cast<std::size_t>(reference_points_));

  std::vector<TrajectoryPoint> out;
  out.reserve(end - start);

  for (std::size_t i = start; i < end; ++i) {
    const auto & tp = traj.points[i];

    TrajectoryPoint pt;
    pt.x = tp.pose.position.x;
    pt.y = tp.pose.position.y;
    pt.psi = yawFromPose(tp.pose);

    const double target_v = static_cast<double>(tp.longitudinal_velocity_mps);

    // Prefer the velocity profiler's target speed. Fall back only when the point velocity is invalid.
    if (std::isfinite(target_v) && target_v >= 0.0) {
      pt.v = target_v;
    } else {
      pt.v = controller_params_.reference_speed;
    }

    // Preserve stop metadata so the controller can enforce final stop behavior without knowing behavior logic.
    pt.is_stop_point = tp.is_stop_point;
    pt.stop_reason = tp.stop_reason;

    out.push_back(pt);
  }

  return out;
}

std::size_t ControllerNode::findNearestIndex(
  const planning_msgs::msg::Trajectory & traj, const VehicleState & state,
  std::size_t start_from) const
{
  std::size_t nearest_index = start_from;
  double nearest_dist_sq = std::numeric_limits<double>::max();

  // Linear nearest-neighbor search is acceptable for short local trajectories.
  // If trajectories become large, consider caching progress or using a bounded search window.
  for (std::size_t i = start_from; i < traj.points.size(); ++i) {
    const auto & p = traj.points[i].pose.position;
    const double dx = p.x - state.x;
    const double dy = p.y - state.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < nearest_dist_sq) {
      nearest_dist_sq = d2;
      nearest_index = i;
    }
  }

  return nearest_index;
}

VehicleState ControllerNode::odomToState(const nav_msgs::msg::Odometry & odom) const
{
  const auto & pose = odom.pose.pose;
  const double yaw = yawFromPose(pose);

  const double vx = odom.twist.twist.linear.x;
  const double vy = odom.twist.twist.linear.y;
  return VehicleState{pose.position.x, pose.position.y, yaw, std::hypot(vx, vy)};
}

void ControllerNode::publishZero(double brake)
{
  // Common safe command used for stale data, invalid controller output, short trajectory, or hard stop.
  carla_msgs::msg::CarlaEgoVehicleControl command;
  command.header.stamp = now();
  command.header.frame_id = latest_odometry_ ? latest_odometry_->header.frame_id : "map";
  command.throttle = 0.0F;
  command.brake = static_cast<float>(std::clamp(brake, 0.0, 1.0));
  command.steer = 0.0F;
  command.hand_brake = false;
  command.reverse = false;
  command.gear = 1;
  control_pub_->publish(command);

  // Reset stateful filters and steering memory so the next valid command starts from a known safe state.
  previous_front_steer_ = 0.0;
  filtered_steer_ = 0.0;
  filtered_accel_ = 0.0;
}

bool ControllerNode::dataValid() const
{
  // Both trajectory and odometry are required for closed-loop control.
  if (!latest_trajectory_ || !latest_odometry_) {
    return false;
  }

  const auto now_t = now();
  const double trajectory_age = (now_t - trajectory_stamp_).seconds();
  const double odometry_age = (now_t - odometry_stamp_).seconds();

  // Reject stale inputs to avoid driving on outdated route or ego-state information.
  if (trajectory_age > trajectory_timeout_ || odometry_age > odometry_timeout_) {
    return false;
  }

  // Reject non-finite odometry: a NaN/Inf pose or twist would otherwise propagate
  // through the filters and reach the published command.
  const auto & odom_pose = latest_odometry_->pose.pose.position;
  const auto & odom_twist = latest_odometry_->twist.twist.linear;
  if (
    !std::isfinite(odom_pose.x) || !std::isfinite(odom_pose.y) ||
    !std::isfinite(odom_twist.x) || !std::isfinite(odom_twist.y))
  {
    return false;
  }

  // Reject non-finite trajectory points for the same reason.
  for (const auto & point : latest_trajectory_->points) {
    if (!std::isfinite(point.pose.position.x) || !std::isfinite(point.pose.position.y)) {
      return false;
    }
  }

  return true;
}

double ControllerNode::controlPeriod() const { return 1.0 / publish_rate_; }

}  // namespace controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<controller::ControllerNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller_node"), "Fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
