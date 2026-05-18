# Velocity Profiler

## Overview

This package implements the velocity profile generation layer for the autonomous driving planning stack.

It receives a geometric trajectory from the local planner and converts it into a time-parameterized trajectory with velocity, acceleration, and timing information.

The velocity profiler uses:

- local planner trajectory points
- per-point speed limits from the local planner
- explicit stop points from the local planner
- ego vehicle odometry
- curvature-based speed reduction
- acceleration and deceleration limits
- jerk smoothing

The output trajectory is published for the controller and can also be visualized/debugged in RViz2.

Published outputs:

- `/velocity_profiler/trajectory`
- `/velocity_profiler/debug/speed_profile`
- `/velocity_profiler/debug/markers`

## Purpose

The goal of this task is to provide a feasible velocity profile for the controller.

The velocity profiler is responsible for longitudinal trajectory timing only.

It does not handle:

- global route planning
- behavior decision making
- obstacle classification
- geometric obstacle avoidance
- steering control
- throttle/brake control

Those responsibilities are handled by other planning stack modules.

## System Design

The package is divided into two main parts:

- pure C++ velocity profiler library
- ROS 2 velocity profiler node

Pipeline:

```text
Local Planner Trajectory + Ego Odometry
        ↓
VelocityProfilerNode
        ↓
VelocityProfiler Library
        ↓
Geometry + Speed Caps + Stop Constraints
        ↓
Forward Pass + Backward Pass + Jerk Smoothing
        ↓
planning_msgs/Trajectory
```

## Velocity profiler library

The core profiler is implemented as a pure C++ class.

Main files:

- `include/velocity_profiler/velocity_profiler.hpp`
- `src/velocity_profiler.cpp`

Responsibilities:

- receive trajectory waypoints from the local planner
- compute waypoint spacing
- estimate curvature from heading changes
- apply per-point velocity caps
- apply curvature-based speed limits
- force zero velocity at stop points
- run acceleration-limited forward pass
- run deceleration-limited backward pass over the whole profile
- smooth the velocity profile using a jerk heuristic
- re-apply the acceleration and deceleration limits
- assign acceleration and `time_from_start`

This keeps the profiling logic independent from ROS 2 communication.

## Velocity profiler node

The ROS 2 node wraps the profiler library.

Main files:

- `include/velocity_profiler/velocity_profiler_node.hpp`
- `src/velocity_profiler_node.cpp`

Responsibilities:

- subscribe to the local planner trajectory
- subscribe to ego vehicle odometry
- convert ROS trajectory messages into internal waypoints
- call the pure C++ profiler
- convert profiled waypoints back to `planning_msgs/Trajectory`
- publish the latest profiled trajectory periodically
- publish debug speed arrays and RViz markers

Subscribed topics:

- `/local_planner/trajectory`
- `/carla/ego_vehicle/odometry`

Published topics:

- `/velocity_profiler/trajectory`
- `/velocity_profiler/debug/speed_profile`
- `/velocity_profiler/debug/markers`

## Profiling Approach

The profiler treats the local planner trajectory as a geometric path with constraints.

Each waypoint carries:

- position
- heading
- maximum allowed velocity
- optional stop-point flag
- optional stop reason

The profiler fills:

- target longitudinal velocity
- longitudinal acceleration
- time from trajectory start

Planning flow:

```text
Local Planner Trajectory
   ↓
Convert to profiler waypoints
   ↓
Compute ds and curvature
   ↓
Compute speed caps
   ↓
Forward acceleration pass
   ↓
Backward deceleration pass
   ↓
Jerk smoothing
   ↓
Assign timing
   ↓
Publish timed trajectory
```

## Input Constraints from the Local Planner

The velocity profiler does not subscribe directly to the behavior planner.

Instead, the local planner encodes behavior constraints into the trajectory points.

Important fields copied from the local planner trajectory:

- `max_velocity_mps`
- `is_stop_point`
- `stop_reason`

This keeps the velocity profiler focused on trajectory timing and avoids coupling it to behavior-state logic.

Examples:

- lane following uses the normal speed cap
- following a leading vehicle uses a lower per-point speed cap
- avoidance can use a behavior-specific speed cap
- traffic-light or goal stopping is represented using `is_stop_point`

## Speed cap calculation

For each waypoint, the profiler computes a velocity cap from multiple constraints.

The cap is the minimum of:

- configured maximum speed
- per-point `max_velocity_mps` from the local planner
- curvature-based speed limit
- configured cruise speed

If a waypoint is marked as a stop point, the cap becomes zero.

Curvature-based speed reduction uses the lateral acceleration relation:

```text
v_curve = sqrt(a_lat_max / |curvature|)
```

This reduces speed in sharp turns and keeps the profile more suitable for vehicle tracking.

## Forward Pass

The forward pass limits acceleration from the current ego speed.

It starts from the current ego velocity received from odometry.

For each next waypoint, the profiler computes the highest velocity reachable under the configured acceleration limit.

This prevents sudden jumps from low speed to high speed.

## Backward Pass

The backward pass enforces stopping and deceleration feasibility.

Stop points come from explicit `is_stop_point` flags. The last waypoint is not treated as a stop, because the trajectory is a receding-horizon window rather than the full route.

The profiler:

- forces zero velocity at each stop point and after it
- sweeps backward over the whole profile so that `v[i]^2 <= v[i+1]^2 + 2 * a_dec * ds`

Sweeping the whole profile, not only from stop points, matters because a drop in the speed cap (curve entry, a speed-limited zone) needs a braking ramp too.

## Jerk Smoothing

After the forward and backward passes, the profiler applies a simple jerk-smoothing heuristic.

The smoother checks acceleration changes between neighboring trajectory segments and lowers velocities when the estimated jerk exceeds the configured limit.

Lowering speeds can break the acceleration or braking bound, so the forward and backward limit passes run again afterwards. The result always satisfies `a_max` and `a_dec`; `j_max` is best effort. `tests/test_velocity_profiler.cpp` checks both bounds on every pair of points.

## Timing Assignment

After velocity and acceleration are assigned, the profiler computes `time_from_start` for each waypoint.

Timing is estimated using segment distance and average velocity between consecutive waypoints.

The final trajectory contains:

- `longitudinal_velocity_mps`
- `acceleration_mps2`
- `time_from_start`

These fields are used by the controller.

## Single-Point Stop Handling

The node handles a special case where the local planner publishes a single stop point.

If the received trajectory contains one point and that point is marked as `is_stop_point`, the node forwards it as a zero-speed trajectory instead of rejecting it.

This supports fallback stop behavior from the local planner.

## ROS 2 Interfaces

### Subscribed Topics

| Topic                           | Type                             | Description                                    |
| ------------------------------- | -------------------------------- | ---------------------------------------------- |
| `/local_planner/trajectory`   | `planning_msgs/msg/Trajectory` | Geometric trajectory from the local planner    |
| `/carla/ego_vehicle/odometry` | `nav_msgs/msg/Odometry`        | Ego vehicle velocity used as the initial speed |

### Published Topics

| Topic                                      | Type                                   | Description                                      |
| ------------------------------------------ | -------------------------------------- | ------------------------------------------------ |
| `/velocity_profiler/trajectory`          | `planning_msgs/msg/Trajectory`       | Time-parameterized trajectory for the controller |
| `/velocity_profiler/debug/speed_profile` | `std_msgs/msg/Float32MultiArray`     | Debug array containing waypoint velocities       |
| `/velocity_profiler/debug/markers`       | `visualization_msgs/msg/MarkerArray` | RViz marker visualization of the speed profile   |

## Parameters

Main parameters are loaded from:

```text
config/velocity_profiler_params.yaml
```

Important parameters:

| Parameter                   | Description                                                  |
| --------------------------- | ------------------------------------------------------------ |
| `map_frame`               | Frame used for visualization                                 |
| `cruise_speed`            | Nominal target speed                                         |
| `max_speed`               | Absolute maximum speed                                       |
| `a_max`                   | Maximum longitudinal acceleration                            |
| `a_dec`                   | Maximum longitudinal deceleration                            |
| `j_max`                   | Maximum jerk used by the smoothing heuristic                 |
| `a_lat_max`               | Maximum lateral acceleration used for curvature speed limits |
| `min_curvature_threshold` | Curvature threshold below which curvature is ignored         |
| `min_ds`                  | Minimum segment distance for numerical stability             |

The node relies on constraints encoded in the trajectory by the local planner rather than subscribing to behavior states. `trajectory_timeout` (default 0.5 s) drops the last profile if no new trajectory arrives, so a stale profile is never republished.

## Package Structure

```text
velocity_profiler/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── velocity_profiler_params.yaml
├── include/velocity_profiler/
│   ├── velocity_profiler.hpp
│   └── velocity_profiler_node.hpp
├── launch/
│   └── velocity_profiler.launch.py
└── src/
    ├── velocity_profiler.cpp
    └── velocity_profiler_node.cpp
```

## Dependencies

Tested environment:

- Ubuntu 22.04
- ROS 2 Humble
- CARLA 0.9.16
- C++17

ROS dependencies:

- `rclcpp`
- `nav_msgs`
- `geometry_msgs`
- `std_msgs`
- `visualization_msgs`
- `planning_msgs`
- `tf2`
- `tf2_ros`
- `tf2_geometry_msgs`
- `ament_cmake`

## Build

From the workspace root:

```bash
colcon build --packages-up-to velocity_profiler
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

Launch the upstream planning nodes so that `/local_planner/trajectory` is available.

Then launch the velocity profiler:

```bash
ros2 launch velocity_profiler velocity_profiler.launch.py
```

Or pass a custom parameter file:

```bash
ros2 launch velocity_profiler velocity_profiler.launch.py \
  params_file:=/absolute/path/to/velocity_profiler_params.yaml
```

## RViz2 Workflow

Open RViz2.

Set the fixed frame to:

```text
map
```

Useful topics to visualize:

- `/velocity_profiler/debug/markers`
- `/velocity_profiler/trajectory`

The debug markers show trajectory points with speed-based coloring and elevated markers for stop points.

## Debugging

Useful commands:

```bash
ros2 topic echo /local_planner/trajectory
ros2 topic echo /velocity_profiler/trajectory
ros2 topic echo /velocity_profiler/debug/speed_profile
ros2 topic info /velocity_profiler/trajectory -v
```

Check that the local planner trajectory contains valid speed and stop fields:

```bash
ros2 topic echo /local_planner/trajectory --field points
```

Important fields to inspect:

- `max_velocity_mps`
- `is_stop_point`
- `stop_reason`
- `longitudinal_velocity_mps`
- `acceleration_mps2`
- `time_from_start`
