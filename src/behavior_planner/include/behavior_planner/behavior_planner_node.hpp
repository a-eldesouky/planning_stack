#ifndef BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_NODE_HPP_
#define BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_NODE_HPP_

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include <carla_msgs/msg/carla_traffic_light_info_list.hpp>
#include <carla_msgs/msg/carla_traffic_light_status_list.hpp>
#include <cstdint>
#include <derived_object_msgs/msg/object_array.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <planning_msgs/msg/behavior_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_planner/behavior_context.hpp"
#include "planning_msgs/msg/route.hpp"

namespace behavior_planner
{

/**
 * @class BehaviorPlannerNode
 * @brief ROS 2 node that runs the behavior planner behavior tree.
 *
 * BehaviorPlannerNode receives the global route, ego odometry, detected objects,
 * and CARLA traffic light information. It updates a shared BehaviorContext,
 * ticks the BehaviorTree.CPP tree periodically, and publishes the selected
 * behavior command for downstream planning modules.
 *
 * Main responsibilities:
 * - subscribe to route, odometry, objects, and traffic light topics
 * - maintain shared behavior context
 * - register BehaviorTree.CPP condition and action nodes
 * - tick the behavior tree at a fixed rate
 * - publish planning_msgs::msg::BehaviorCommand
 * - optionally expose Groot2 live visualization
 */
class BehaviorPlannerNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the behavior planner node.
   *
   * The constructor initializes parameters, subscriptions, publisher,
   * behavior tree nodes, behavior tree XML, and the periodic tick timer.
   *
   * @param options ROS 2 node options.
   */
  explicit BehaviorPlannerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief Declare ROS parameters and load them into the node/context.
   */
  void declareAndLoadParams();

  /**
   * @brief Create all ROS 2 subscriptions used by the behavior planner.
   */
  void createSubscriptions();

  /**
   * @brief Create the behavior command publisher.
   */
  void createPublisher();

  /**
   * @brief Register custom BehaviorTree.CPP condition and action nodes.
   */
  void registerBTNodes();

  /**
   * @brief Load the behavior tree XML file and optionally enable Groot2 monitoring.
   */
  void loadBehaviorTree();

  /**
   * @brief Create the periodic timer that ticks the behavior tree.
   */
  void createTickTimer();

  /**
   * @brief Store the latest global route from the mission planner.
   *
   * @param msg Route message containing global route waypoints and metadata.
   */
  void routeCallback(const planning_msgs::msg::Route::ConstSharedPtr & msg);

  /**
   * @brief Store the latest ego vehicle odometry.
   *
   * @param msg Ego odometry message.
   */
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  /**
   * @brief Parse and store the latest detected objects from CARLA.
   *
   * @param msg CARLA / derived object array message.
   */
  void objectsCallback(const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg);

  /**
   * @brief Store the latest CARLA traffic light status information.
   *
   * @param msg CARLA traffic light status list.
   */
  void trafficLightStatusCallback(
    const carla_msgs::msg::CarlaTrafficLightStatusList::ConstSharedPtr & msg);

  /**
   * @brief Store the latest CARLA traffic light position information.
   *
   * @param msg CARLA traffic light info list.
   */
  void trafficLightInfoCallback(
    const carla_msgs::msg::CarlaTrafficLightInfoList::ConstSharedPtr & msg);

  /**
   * @brief Convert CARLA detected objects into simplified TrackedObject entries.
   *
   * @param msg CARLA / derived object array message.
   * @return Simplified tracked objects used by the behavior planner.
   */
  std::vector<TrackedObject> parseObjects(
    const derived_object_msgs::msg::ObjectArray::ConstSharedPtr & msg);

  /**
   * @brief Fuse traffic light status and position maps into BehaviorContext entries.
   *
   * @note Must be called while holding ctx_->mtx.
   */
  void fuseTrafficLights();

  /**
   * @brief Periodic behavior planner update.
   *
   * 1. Acquires lock briefly to update derived state (leading vehicle,
   *    traffic light, hysteresis).
   * 2. Releases lock: callbacks are unblocked.
   * 3. Ticks the BT without holding the lock (safe: BT nodes read only
   *    derived fields; see ConditionBase::ctx_ for the full invariant).
   * 4. Acquires lock once more to capture current_command, then publishes.
   *    Single acquisition ensures the published command matches the BT output.
   */
  void tick();

  /**
   * @brief Shared behavior planner context.
   *
   * ROS callbacks update this context, condition nodes read it, action nodes
   * write behavior decisions into it, and the node publishes the final command.
   */
  std::shared_ptr<BehaviorContext> ctx_;

  /**
   * @brief BehaviorTree.CPP factory used to register custom BT nodes.
   */
  BT::BehaviorTreeFactory bt_factory_;

  /**
   * @brief Loaded behavior tree instance.
   */
  BT::Tree bt_tree_;

  /**
   * @brief Enables live Groot2 monitoring when true.
   */
  bool enable_groot_monitoring_{false};

  /**
   * @brief Groot2 publisher used for live behavior tree visualization.
   */
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;

  /// Subscription to the mission planner global route.
  rclcpp::Subscription<planning_msgs::msg::Route>::SharedPtr route_sub_;

  /// Subscription to ego vehicle odometry.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  /// Subscription to detected objects.
  rclcpp::Subscription<derived_object_msgs::msg::ObjectArray>::SharedPtr objects_sub_;

  /// Subscription to CARLA traffic light status messages.
  rclcpp::Subscription<carla_msgs::msg::CarlaTrafficLightStatusList>::SharedPtr tl_status_sub_;

  /// Subscription to CARLA traffic light info / position messages.
  rclcpp::Subscription<carla_msgs::msg::CarlaTrafficLightInfoList>::SharedPtr tl_info_sub_;

  /// Publisher for the selected behavior command.
  rclcpp::Publisher<planning_msgs::msg::BehaviorCommand>::SharedPtr cmd_pub_;

  /// Timer used to periodically tick the behavior tree.
  rclcpp::TimerBase::SharedPtr tick_timer_;

  /// Timestamp of the last received objects message.
  /// Used to clear stale objects when the publisher stops.
  /// Timestamp of the last received objects message. Not initialized until
  /// the first message arrives: use objects_ever_received_ as guard.
  rclcpp::Time last_objects_stamp_;

  /// True after the first ObjectArray message has been received.
  bool objects_ever_received_{false};

  /// Objects are considered stale after this duration with no new message [s].
  static constexpr double kObjectsTimeoutSec{1.0};

  /// Topic name for the global route input.
  std::string global_route_topic_;

  /// Topic name for ego odometry input.
  std::string odometry_topic_;

  /// Topic name for object detection input.
  std::string objects_topic_;

  /// Topic name for CARLA traffic light status input.
  std::string tl_status_topic_;

  /// Topic name for CARLA traffic light info input.
  std::string tl_info_topic_;

  /// Topic name for behavior command output.
  std::string behavior_command_topic_;

  /// Path to the BehaviorTree.CPP XML file.
  std::string behavior_tree_file_;

  /**
   * @brief Latest traffic light states indexed by traffic light ID.
   *
   * @note Accessed together with ctx_->mtx.
   */
  std::unordered_map<uint32_t, uint8_t> tl_status_map_;

  /**
   * @brief Latest traffic light positions indexed by traffic light ID.
   *
   * Stored as:
   *
   * @code
   * id -> (x, y)
   * @endcode
   *
   * @note Accessed together with ctx_->mtx.
   */
  std::unordered_map<uint32_t, std::pair<double, double>> tl_position_map_;

  /**
   * @brief Register a custom BehaviorTree.CPP node type.
   *
   * All behavior tree nodes in this package share the same constructor shape:
   *
   * @code
   * NodeT(name, config, ctx_, logger, clock)
   * @endcode
   *
   * @tparam NodeT Behavior tree condition/action node type.
   * @param registration_name Name used in the behavior tree XML file.
   */
  template<typename NodeT>
  void registerBTNode(const std::string & registration_name)
  {
    bt_factory_.registerBuilder<NodeT>(
      registration_name, [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<NodeT>(name, config, ctx_, get_logger(), get_clock());
      });
  }
};

}  // namespace behavior_planner

#endif  // BEHAVIOR_PLANNER__BEHAVIOR_PLANNER_NODE_HPP_
