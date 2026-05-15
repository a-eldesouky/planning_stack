# Behavior Planner

## Overview

This package implements the behavior planning layer for the autonomous driving planning stack.

It selects the high-level driving behavior that should be executed by the downstream local planner and velocity planner.

The behavior planner uses:

- Global route from the mission planner
- CARLA ego vehicle odometry
- CARLA object detections
- CARLA traffic light status and information
- BehaviorTree.CPP for behavior arbitration
- Optional Groot2 live monitoring

Published output:

- `/behavior_planner/behavior_command`

## Purpose

The goal of this task is to decide what the vehicle should do next at the behavior level.

The behavior planner is responsible for selecting between:

- lane following
- following a moving leading vehicle
- avoiding a static leading obstacle
- waiting behind a static obstacle
- stopping at a red or yellow traffic light

It does not generate the final geometric trajectory.

It does not perform:

- global route search
- Hybrid A* planning
- velocity profile generation
- low-level trajectory tracking

Those responsibilities are handled by the mission planner, local planner, velocity profile generator, and controller.

## System Design

The package is divided into four main parts:

- ROS 2 behavior planner node
- shared behavior context
- BehaviorTree.CPP condition nodes
- BehaviorTree.CPP action nodes

Pipeline:

```text
Mission Route + Odometry + Objects + Traffic Lights
        ↓
BehaviorPlannerNode
        ↓
BehaviorContext
        ↓
BehaviorTree.CPP Main Tree
        ↓
Selected BehaviorCommand
        ↓
Local Planner / Velocity Planner
```

## Behavior planner node

The ROS 2 node owns the subscriptions, publisher, behavior tree factory, and tick timer.

Main files:

- `include/behavior_planner/behavior_planner_node.hpp`
- `src/behavior_planner_node.cpp`

Responsibilities:

- load planner parameters
- subscribe to route, odometry, objects, and traffic light topics
- fuse traffic light status with traffic light position
- update derived behavior state
- register custom BehaviorTree.CPP nodes
- load the behavior tree XML file
- tick the behavior tree at a fixed rate
- publish `planning_msgs/msg/BehaviorCommand`
- optionally expose the tree to Groot2

## Behavior Context

The behavior context acts as the shared blackboard between ROS callbacks and behavior tree nodes.

Main files:

- `include/behavior_planner/behavior_context.hpp`: declarations, enums, structs, ConditionView, ActionView
- `src/behavior_context.cpp`: all method implementations

It stores:

- latest ego odometry
- latest global route
- filtered objects
- fused traffic light entries
- selected leading vehicle
- relevant traffic light state
- goal reached flag
- hysteresis counters
- current behavior command
- planner parameters

The context also provides helper logic for:

- ego yaw extraction
- ego speed calculation
- distance-to-point calculation
- forward projection distance
- lateral distance to route
- leading vehicle selection
- relevant traffic light selection
- goal reached detection
- hysteresis updates

### Narrow access interfaces

Condition and action nodes access the context through two narrow view classes rather than the raw shared pointer:

- **`ConditionView`**: read-only access to all context fields, plus named mutators for avoidance latch state only. Prevents conditions from accidentally writing fields they don't own.
- **`ActionView`**: write access to `current_command` only, plus read access to the small set of fields actions need to populate it. Prevents actions from writing any other context field.

## Behavior Tree

The behavior tree is defined in:

- `behavior_trees/main_tree.xml`

The tree uses a priority-ordered fallback structure.

Priority order:

1. Controlled T-junction / traffic light handling
2. Avoid static leading vehicle
3. Wait behind static obstacle
4. Follow moving leading vehicle
5. Follow lane

Tree structure:

```text
BehaviorRoot
├── ControlledTJunctionHandling
│   ├── IsAtTJunction
│   ├── IsTrafficLightRedOrYellow
│   └── StopAtTrafficLight
├── AvoidStaticLeadingVehicle
│   ├── IsLeadingVehicleDetected
│   ├── IsLeadingVehicleStatic
│   ├── IsAvoidancePossible
│   └── RequestAvoidance
├── WaitBehindStaticObstacle
│   ├── IsLeadingVehicleDetected
│   ├── IsLeadingVehicleStatic
│   └── WaitBehindObstacle
├── FollowLeadingVehicle
│   ├── IsLeadingVehicleDetected
│   ├── IsLeadingVehicleMoving
│   └── FollowLeadingVehicle
└── FollowLane
```

The fallback tree ensures that safety-critical behaviors have higher priority than normal lane following.

## Condition Nodes

Condition nodes read the shared behavior context via `ConditionView` and return `SUCCESS` or `FAILURE`.

Main files:

- `include/behavior_planner/conditions.hpp`
- `src/conditions.cpp`

Implemented condition nodes:

| Condition                     | Purpose                                                                           |
| ----------------------------- | --------------------------------------------------------------------------------- |
| `IsAtTJunction`             | Checks if the ego vehicle is near a relevant controlled junction or traffic light |
| `IsTrafficLightRedOrYellow` | Checks if the stable relevant traffic light state requires stopping               |
| `IsLeadingVehicleDetected`  | Checks if a stable leading object exists ahead on the route                       |
| `IsLeadingVehicleStatic`    | Checks if the leading object is confirmed static                                  |
| `IsLeadingVehicleMoving`    | Checks if the leading object is moving                                            |
| `IsAvoidancePossible`       | Checks if there is enough distance to request avoidance                           |

## Action Nodes

Action nodes write the selected behavior command via `ActionView`.

Main files:

- `include/behavior_planner/actions.hpp`
- `src/actions.cpp`

Implemented action nodes:

| Action                   | Behavior command                               |
| ------------------------ | ---------------------------------------------- |
| `StopAtTrafficLight`   | Stop for red or yellow traffic light           |
| `RequestAvoidance`     | Request static obstacle avoidance              |
| `WaitBehindObstacle`   | Stop and wait behind a static obstacle         |
| `FollowLeadingVehicle` | Follow a moving leading vehicle with speed cap |
| `FollowLane`           | Normal lane following                          |

## Behavior Modes

The package maps the selected behavior to `planning_msgs/msg/BehaviorCommand`.

Behavior values:

| Value | Mode                       |
| ----: | -------------------------- |
|     0 | `FOLLOW_LANE`            |
|     1 | `FOLLOW_LEADING_VEHICLE` |
|     2 | `AVOID_STATIC_OBSTACLE`  |
|     3 | `STOP_AT_TRAFFIC_LIGHT`  |
|     4 | `WAIT_BEHIND_OBSTACLE`   |

Each command includes:

- header
- behavior mode
- target speed
- safe distance
- reason string

## Leading vehicle detection

Objects are received from CARLA and converted into simplified tracked objects.

The planner filters objects using:

- ego filter radius
- forward distance from ego
- detection range
- lateral distance to the global route
- object speed

The closest valid object ahead of the ego vehicle is selected as the leading vehicle.

A moving leading vehicle triggers following behavior.

A static leading vehicle triggers either avoidance or waiting, depending on avoidance feasibility.

## Static obstacle handling

Static obstacles are detected using a speed threshold.

If the leading object speed is below `static_speed_threshold`, it is treated as static after hysteresis confirmation.

If avoidance is enabled and the object is far enough ahead, the behavior planner requests avoidance.

If avoidance is not possible, the behavior planner commands the vehicle to wait behind the obstacle.

## Traffic light handling

Traffic light behavior uses two CARLA inputs:

- traffic light status
- traffic light info / position

The node fuses both streams by traffic light ID.

The relevant traffic light is selected using:

- forward position relative to ego
- distance to ego
- ego heading cone
- lateral proximity to the route

Red and yellow lights produce a stop behavior after hysteresis confirmation.

Green or unknown lights allow the behavior tree to continue to the lower-priority branches.

## Hysteresis

The behavior planner uses hysteresis counters to avoid flickering behavior decisions.

Stable flags are generated for:

- leading vehicle detection
- static obstacle classification
- traffic light stop condition

Relevant parameters:

- `leading_vehicle_confirm_ticks`
- `static_obstacle_confirm_ticks`
- `traffic_light_confirm_ticks`
- `exit_hysteresis_ticks`

This prevents one-frame perception noise from immediately changing the selected behavior.

## Groot2 Monitoring

The behavior planner can expose the running BehaviorTree.CPP tree to Groot2.

This is controlled by:

```yaml
enable_groot_monitoring: true
```

When enabled, the node creates a Groot2 publisher after loading the behavior tree.

Use Groot2 to visualize which branch of the tree is active while the vehicle is running.

## ROS 2 Interfaces

### Subscribed Topics

| Topic                            | Type                                           | Description               |
| -------------------------------- | ---------------------------------------------- | ------------------------- |
| `/mission_planner/route`       | `planning_msgs/msg/Route`                    | Global route from the mission planner  |
| `/carla/ego_vehicle/odometry`  | `nav_msgs/msg/Odometry`                      | Current ego vehicle state |
| `/carla/objects`               | `derived_object_msgs/msg/ObjectArray`        | CARLA detected objects    |
| `/carla/traffic_lights/status` | `carla_msgs/msg/CarlaTrafficLightStatusList` | Traffic light states      |
| `/carla/traffic_lights/info`   | `carla_msgs/msg/CarlaTrafficLightInfoList`   | Traffic light positions   |

### Published Topics

| Topic                                  | Type                                  | Description                               |
| -------------------------------------- | ------------------------------------- | ----------------------------------------- |
| `/behavior_planner/behavior_command` | `planning_msgs/msg/BehaviorCommand` | Selected behavior for downstream planners |

## Parameters

Main parameters are stored in:

- `config/behavior_planner_params.yaml`

Important parameters:

| Parameter                              |  Default | Description                                   |
| -------------------------------------- | -------: | --------------------------------------------- |
| `tick_rate_hz`                       | `10.0` | Behavior tree tick rate                       |
| `normal_speed`                       |  `8.0` | Target speed for lane following               |
| `following_speed`                    |  `4.0` | Speed cap when following a moving vehicle     |
| `avoidance_speed`                    |  `3.0` | Speed target during avoidance                 |
| `leading_vehicle_detection_distance` | `20.0` | Maximum forward detection distance            |
| `safe_following_distance`            | `10.0` | Desired following gap                         |
| `route_lateral_threshold`            |  `3.5` | Maximum lateral distance from route           |
| `static_speed_threshold`             |  `0.5` | Speed below which an object is static         |
| `ego_filter_radius`                  |  `2.5` | Radius used to ignore ego self-detections     |
| `junction_detection_distance`        | `12.0` | Distance threshold for traffic light handling |
| `traffic_light_stop_distance`        | `10.0` | Desired stop distance near traffic light      |
| `enable_avoidance`                   | `true` | Enables static obstacle avoidance requests    |
| `minimum_avoidance_distance`         |  `3.0` | Minimum runway distance for avoidance         |
| `road_boundary_margin`               |  `0.5` | Safety margin from road edge                  |
| `goal_reached_threshold`             |  `3.0` | Distance to route end considered goal reached |

## Package Structure

```text
behavior_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── behavior_trees/
│   └── main_tree.xml
├── config/
│   └── behavior_planner_params.yaml
├── include/behavior_planner/
│   ├── actions.hpp
│   ├── behavior_context.hpp
│   ├── behavior_planner_node.hpp
│   └── conditions.hpp
├── launch/
│   └── behavior_planner.launch.py
└── src/
    ├── actions.cpp
    ├── behavior_context.cpp
    ├── behavior_planner_node.cpp
    └── conditions.cpp
```

## Dependencies

Tested environment:

- Ubuntu 22.04
- ROS 2 Humble
- CARLA 0.9.16
- C++17
- BehaviorTree.CPP v4

ROS dependencies:

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `ament_index_cpp`
- `planning_msgs`
- `behaviortree_cpp`
- `carla_msgs`
- `derived_object_msgs`
- `ament_cmake`

## Build

From the workspace root:

```bash
colcon build --packages-up-to behavior_planner
source install/setup.bash
```

## Run

Start CARLA first:

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh
```

Start the CARLA ROS bridge with the ego vehicle:

```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

Launch the mission planner and publish a route from RViz2.

Then launch the behavior planner:

```bash
ros2 launch behavior_planner behavior_planner.launch.py
```

## RViz2 Workflow

1. Start CARLA and the CARLA ROS bridge.
2. Launch the mission planner.
3. Open RViz2.
4. Set the fixed frame to `map`.
5. Use the `2D Goal Pose` tool to publish a goal.
6. Confirm that `/mission_planner/route` is published.
7. Launch the behavior planner.
8. Echo `/behavior_planner/behavior_command` to inspect the selected behavior.

Useful command:

```bash
ros2 topic echo /behavior_planner/behavior_command
```

## Debugging Commands

Check object input:

```bash
ros2 topic info /carla/objects -v
ros2 topic echo /carla/objects --once
```

Check traffic light inputs:

```bash
ros2 topic echo /carla/traffic_lights/status --once
ros2 topic echo /carla/traffic_lights/info --once
```

Check selected behavior:

```bash
ros2 topic echo /behavior_planner/behavior_command
```

## Expected Behavior

Normal case:

```text
No obstacle + no red/yellow traffic light → FOLLOW_LANE
```

Moving leading vehicle:

```text
Vehicle ahead + speed >= static threshold → FOLLOW_LEADING_VEHICLE
```

Static obstacle with enough distance:

```text
Static obstacle ahead + avoidance enabled + enough runway → AVOID_STATIC_OBSTACLE
```

Static obstacle without enough distance:

```text
Static obstacle ahead + avoidance not possible → WAIT_BEHIND_OBSTACLE
```

Red or yellow traffic light:

```text
Relevant traffic light ahead + red/yellow stable → STOP_AT_TRAFFIC_LIGHT
```
