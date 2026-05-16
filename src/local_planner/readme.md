# Local Planner

## Overview

This package implements the local planning layer for the autonomous driving planning stack.

It converts the mission planner global route into a short-horizon local trajectory that can be consumed by the velocity profile generator and controller.

The local planner uses:

- Mission planner route waypoints
- CARLA ego vehicle odometry
- CARLA detected objects
- Behavior planner commands
- Hybrid A* for static obstacle avoidance
- Cubic spline smoothing for local trajectory quality

Published outputs:

- `/local_planner/trajectory`
- `/local_planner/debug/trajectory`

## Purpose

The goal of this task is to generate a safe and locally feasible trajectory from the global route.

The local planner is responsible for:

- following the route from the mission planner
- respecting behavior decisions from the behavior planner
- generating short-horizon trajectory points
- applying per-point speed constraints
- marking stop points for downstream velocity planning
- planning around static obstacles when avoidance is requested
- keeping avoidance maneuvers inside the route road corridor

It does not handle:

- global route search
- behavior decision making
- velocity time-parameterization
- throttle, brake, or steering control

## System Design

The package is divided into five parts:

1. Pure C++ Hybrid A* planner (`hybrid_astar`)
2. Pure C++ cubic spline smoother (`cubic_spline`)
3. Obstacle processor helper (`obstacle_processor`)
4. Avoidance latch state machine (`avoidance_latch`)
5. Stop latch state machine (`stop_latch`)
6. ROS 2 local planner node (`local_planner_node`)

The three helper classes own all business logic and are independently testable without ROS. The node is a thin ROS wrapper that delegates to them.

Pipeline:

```text
Mission Planner Route + Ego Odometry + Objects + Behavior Command
        ↓
LocalPlannerNode
        ↓
ObstacleProcessor → filtered obstacle circles
        ↓
Route Progress Tracking + Behavior Constraint Mapping
        ↓
Route Interpolation or Hybrid A* (via AvoidanceLatch)
        ↓
Stop Latch enforcement (via StopLatch)
        ↓
Optional Cubic Spline Smoothing
        ↓
planning_msgs/Trajectory + nav_msgs/Path Debug Output
```

## Helper Classes

### ObstacleProcessor

Converts raw CARLA object messages into inflated obstacle circles. Handles ego-vehicle filtering, 3-circle decomposition of long objects, longitudinal/lateral gating, and `obstacleAhead()` queries.

### AvoidanceLatch

Manages the committed Hybrid A* path state machine. Once a valid avoidance path is found, it is latched and followed until the obstacle is confirmed passed. Includes failure counting and auto-release after `kMaxFailures = 20` consecutive A* failures.

### StopLatch

Manages the committed stop-point state machine. When a stopping behavior brings the ego within `stop_commit_distance` of a stop point, the location is latched to prevent one-cycle behavior dropouts from re-accelerating the vehicle.

## Hybrid A* Planner

Hybrid A* is used for kinematically feasible local obstacle avoidance.

The planner uses:

- continuous vehicle pose
- discretized 3D grid (x, y, heading)
- bicycle model arc integration
- steering angle samples
- obstacle collision checking with vehicle footprint
- route corridor boundary checking
- Dubins-path heuristic (all 6 variants: LSL, RSR, LSR, RSL, RLR, LRL)

Grid key packing uses 16 bits per field (x, y, heading), supporting up to 65535 heading bins without aliasing.

## Collision Checking

Obstacle collision checking uses inflated circular obstacles.

Objects are decomposed into up to 3 circles aligned with the ego heading. The ego heading is used rather than the obstacle's own yaw because CARLA often reports obstacle yaw as identity (0°) regardless of actual orientation.

Hybrid A* checks the vehicle footprint using multiple samples along the vehicle body.

## Route corridor constraint

The local planner builds a route corridor from the global route waypoints using `road_half_width` metadata from the mission planner. This lets the planner cross into the opposing lane during avoidance while staying within the road boundary.

## Cubic spline smoothing

The planner can smooth generated paths using a natural cubic spline. The smoothed result is safety-checked before use; if it fails validation the raw path is used instead. Smoothing is disabled for avoidance paths, since the spline may cut through inflated obstacle circles.

## ROS 2 Interfaces

### Subscribed Topics

| Topic                                  | Type                                    | Description                   |
| -------------------------------------- | --------------------------------------- | ----------------------------- |
| `/mission_planner/route`             | `planning_msgs/msg/Route`             | Global route from the mission planner      |
| `/carla/ego_vehicle/odometry`        | `nav_msgs/msg/Odometry`               | Current ego vehicle pose      |
| `/carla/objects`                     | `derived_object_msgs/msg/ObjectArray` | Detected objects from CARLA   |
| `/behavior_planner/behavior_command` | `planning_msgs/msg/BehaviorCommand`   | Behavior decision from the behavior planner |

### Published Topics

| Topic                               | Type                             | Description                                |
| ----------------------------------- | -------------------------------- | ------------------------------------------ |
| `/local_planner/trajectory`       | `planning_msgs/msg/Trajectory` | Local trajectory for the velocity profiler |
| `/local_planner/debug/trajectory` | `nav_msgs/msg/Path`            | Debug path visualization for RViz2         |

## Parameters

The main parameter file is:

```text
config/local_planner_params.yaml
```

### Vehicle Model

```yaml
vehicle:
  lf: 0.7725
  lr: 0.7725
  wheelbase: 1.545
  width: 1.45
  length: 2.3
  max_steering_angle: 0.2792
  max_steering_rate: 0.17
  min_turning_radius: 5.39
```

### Input Topics

```yaml
input:
  route_topic: "/mission_planner/route"
  odometry_topic: "/carla/ego_vehicle/odometry"
  objects_topic: "/carla/objects"
  behavior_command_topic: "/behavior_planner/behavior_command"
```

### Output Topics

```yaml
output:
  trajectory_topic: "/local_planner/trajectory"
  debug_trajectory_topic: "/local_planner/debug/trajectory"
  publish_rate: 10.0
  frame_id: "map"
```

### Path Following

```yaml
path_following:
  enable_smoothing: true
  lookahead_distance: 30.0
  min_waypoint_spacing: 0.5
  output_points: 100
```

### Route Trimming

```yaml
path_trim:
  search_window_size: 100
  look_behind_points: 2
  max_cross_track_error: 3.0
  goal_reached_threshold: 1.5
```

### Obstacle Detection

```yaml
obstacle_detection:
  lookahead: 30.0
  inflation: 0.8
  corridor_width_multiplier: 1.5
  ego_filter_multiplier: 0.5
  road_boundary_margin: 0.1
  avoidance_corridor_multiplier: 3.0
  min_avoidance_corridor_half_width: 5.2
  avoidance_release_distance: 8.0
```

### Hybrid A*

```yaml
hybrid_astar:
  grid_resolution: 0.5
  heading_bins: 72
  steering_samples: 11
  step_size: 0.8
  max_iterations: 15000
  obstacle_inflation: 0.8
  obstacle_lookahead: 30.0
  heuristic_weight: 1.2
  goal_position_tolerance: 1.5
  goal_heading_tolerance: 0.35
```

### Behavior Constraints

```yaml
behavior:
  stop_creep_speed: 0.5
  stop_creep_distance: 2.0
  stop_commit_distance: 4.0
```

## Package Structure

```text
local_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── local_planner_params.yaml
├── include/local_planner/
│   ├── avoidance_latch.hpp
│   ├── cubic_spline.hpp
│   ├── hybrid_astar.hpp
│   ├── local_planner_node.hpp
│   ├── obstacle_processor.hpp
│   ├── planning_types.hpp
│   └── stop_latch.hpp
├── launch/
│   └── local_planner.launch.py
└── src/
    ├── avoidance_latch.cpp
    ├── cubic_spline.cpp
    ├── hybrid_astar.cpp
    ├── local_planner_node.cpp
    ├── obstacle_processor.cpp
    └── stop_latch.cpp
```

## Dependencies

- Ubuntu 22.04
- ROS 2 Humble
- CARLA 0.9.16
- C++17

ROS dependencies:

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `derived_object_msgs`
- `tf2`
- `tf2_geometry_msgs`
- `planning_msgs`
- `ament_cmake`

## Build

```bash
colcon build --packages-up-to local_planner
source install/setup.bash
```

## Run

Start CARLA first:

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh
```

Start the CARLA ROS bridge:

```bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

Launch the upstream planning modules:

```bash
ros2 launch mission_planner mission_planner.launch.py
ros2 launch behavior_planner behavior_planner.launch.py
```

Launch the local planner:

```bash
ros2 launch local_planner local_planner.launch.py
```

## RViz2 Workflow

Set the fixed frame to `map`. Useful topics to visualize:

- `/mission_planner/route_display`
- `/local_planner/debug/trajectory`

## Expected Behavior

| Behavior command         | Local planner output                                      |
| ------------------------ | --------------------------------------------------------- |
| `FOLLOW_LANE`          | Route-following trajectory at normal speed                |
| `FOLLOW_LEADING_VEHICLE` | Route-following trajectory at reduced speed             |
| `AVOID_STATIC_OBSTACLE`| Hybrid A* trajectory around the obstacle                  |
| `WAIT_BEHIND_OBSTACLE` | Stop trajectory at ego pose                               |
| `STOP_AT_TRAFFIC_LIGHT`| Route-following trajectory with stop point and creep zone |
