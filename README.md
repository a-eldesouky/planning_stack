# Autonomous driving planning stack (ROS 2 + CARLA)

A six-stage autonomous driving planning pipeline, written in C++17 on ROS 2 Humble, running inside the CARLA 0.9.16 simulator on Town01. I built it in 10 days as a personal project to cover the full classical planning hierarchy end to end: HD map generation, mission (global route) planning, behavior arbitration, local trajectory planning, velocity profiling, and closed-loop vehicle control.

## Architecture

```
CARLA Simulator (Town01)
        │
        ▼
┌───────────────────────────────────────────────────────────────────┐
│                         Planning Stack                            │
│                                                                   │
│  HD Map   ──▶  Mission    ──▶  Behavior   ──▶  Local              │
│  (JSON)        Planner         Planner         Planner            │
│                (A*)            (BT.CPP)        (Hybrid A*)        │
│                                    │                │              │
│                               Velocity    ──▶  Controller          │
│                               Profiler         (Pure Pursuit)      │
└───────────────────────────────────────────────────────────────────┘
        │
        ▼
 /carla/ego_vehicle/vehicle_control_cmd
```

| Stage | Package | Approach |
| --- | --- | --- |
| HD map | [`hd_map`](src/hd_map/readme.md) | Lanelet-inspired JSON built from a Town01 point cloud + CARLA semantic data |
| Mission planning | [`mission_planner`](src/mission_planner/readme.md) | A* over the lanelet graph, dynamic start pose, goal from RViz |
| Behavior planning | [`behavior_planner`](src/behavior_planner/readme.md) | BehaviorTree.CPP v4 Fallback tree |
| Local planning | [`local_planner`](src/local_planner/readme.md) | Hybrid A* with a Dubins-path heuristic, cubic-spline smoothing |
| Velocity profiling | [`velocity_profiler`](src/velocity_profiler/readme.md) | Forward acceleration pass, backward braking pass, best-effort jerk smoothing |
| Control | [`controller`](src/controller/readme.md) | Pure Pursuit with speed-scaled dynamic lookahead, PID speed control, kinematic bicycle model |

Each package has its own README with full design notes, ROS interfaces, and parameters. Custom message types live in [`planning_msgs`](src/planning_msgs); the top-level launch file is in [`planning_stack_bringup`](src/planning_stack_bringup).

## What it handles

- Driving end to end from an RViz2 `2D Goal Pose`, with no hard-coded routes.
- Lane following, with transitions to following a leading vehicle or avoiding a stationary one.
- Avoiding a parked (static) vehicle by leaving the lane but staying inside the road boundary, then returning to the global route; if no safe maneuver exists, it holds behind the obstacle instead.
- Stopping on red and yellow at controlled T-junctions, proceeding on green, using live traffic light state from the CARLA ROS bridge (`/carla/traffic_lights/status`, `/carla/traffic_lights/info`).
- A velocity profile that holds the acceleration and braking limits at every point, and Pure Pursuit tracking through all of the above.

Demo recordings for each of these are in [`media/`](media).

## Demo videos

- [Route planning](media/Route%20planning.mp4): end-to-end route planning from an RViz2 goal pose to the vehicle following the global route.
- [Follow leading vehicle](media/Behavior%20Follow%20leading%20vechile.mp4): switches to follow mode behind a moving leading vehicle.
- [Avoid obstacle](media/Behavior%20Avoid%20Obstacle.mp4): avoidance maneuver around a static obstacle.
- [Wait behind](media/Behavior%20Wait%20Behinde.mp4): holds behind a static obstacle when avoidance isn't possible.
- [Red then green](media/Behavior%20Red%20Traffic%20then%20Green.mp4): stops at red, proceeds on green.
- [Yellow then red](media/Behavior%20Yollow%20Traffic%20then%20Red.mp4): decelerates at yellow, stops at red.
- [Intersection, green to red](media/Intersection%20handling%20from%20green%20to%20red.mp4): intersection crossing interrupted by a red light.
- [Intersection, red to green](media/Intersection%20handling%20from%20red%20to%20green.mp4): held at red, releases and drives through on green.
- [Obstacle avoidance](media/Obstacle%20Avoidance.mp4): Hybrid A* avoidance around a parked vehicle, returning to route.
- [Smooth trajectory tracking](media/Smooth%20trajectory%20tracking.mp4): cubic-spline-smoothed trajectory tracked by the controller.

## Repository layout

```
planning_stack/
├── README.md
├── media/                        ← demo videos
├── data/
│   ├── images/                   ← README screenshot
│   └── vehicle_info/             ← vehicle physical parameters
└── src/
    ├── planning_msgs/             ← custom ROS 2 message definitions
    ├── planning_stack_bringup/    ← top-level launch file + RViz2 config
    ├── hd_map/
    ├── mission_planner/
    ├── behavior_planner/
    ├── local_planner/
    ├── velocity_profiler/
    └── controller/
```

`src/ros-bridge` (the CARLA ROS 2 bridge) is not vendored in this repo. See [Build](#build) for cloning it alongside.

## System requirements

| Component    | Version          |
| ------------ | ---------------- |
| OS           | Ubuntu 22.04 LTS |
| ROS 2        | Humble Hawksbill |
| CARLA        | 0.9.16           |
| Python       | 3.10+            |
| C++ standard | C++17            |
| Build tool   | colcon           |

## Build

### 1. Get the CARLA ROS bridge

This stack depends on `carla_msgs`, `carla_ros_bridge`, and `derived_object_msgs` from the official CARLA ROS 2 bridge, which isn't vendored here:

```bash
git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge
cd src/ros-bridge
./install_dependencies.sh
```

### 2. Install ROS 2 dependencies

```bash
sudo apt install -y \
  ros-humble-derived-object-msgs \
  ros-humble-behaviortree-cpp \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-nav-msgs \
  ros-humble-visualization-msgs \
  ros-humble-ament-cmake \
  ros-humble-ament-cmake-gtest \
  python3-colcon-common-extensions \
  python3-rosdep
pip3 install numpy scipy open3d
```

### 3. Get CARLA 0.9.16

Download and extract the CARLA 0.9.16 package for Linux from the [official CARLA releases page](https://github.com/carla-simulator/carla/releases), then install the matching Python API wheel:

```bash
cd <CARLA_ROOT>/PythonAPI/carla/dist
pip3 install carla-0.9.16-cp310-cp310-linux_x86_64.whl
export PYTHONPATH=$PYTHONPATH:<CARLA_ROOT>/PythonAPI/carla
```

### 4. Get the Town01 map data

The HD map generator ([`hd_map/generate_hd_map.py`](src/hd_map/generate_hd_map.py)) needs a Town01 point cloud at `src/hd_map/data/Town01.pcd`.

TODO: the point cloud used during development came from a private share, not a documented public-tool pipeline, so the exact reproduction steps aren't captured here. CARLA ships a semantic LIDAR sensor and a `PythonAPI/util` recording tooling that can be used to export a point cloud of a loaded town. If you regenerate `Town01.pcd` this way, consider sending a PR updating this section with the exact steps.

### 5. Build the workspace

```bash
colcon build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
source install/setup.bash
```

## Run

### One-command launch

```bash
ros2 launch planning_stack_bringup planning_stack.launch.py \
  carla_root:=<CARLA_ROOT>
```

This sequentially starts the CARLA simulator, the CARLA ROS bridge with the example ego vehicle (after 15 s), and all six planning nodes plus RViz2 (after 20 s).

### Manual startup (recommended for demos)

Running each node in its own terminal gives cleaner logs and makes it easier to restart a single node.

```bash
# Terminal 1: CARLA simulator
cd <CARLA_ROOT>
./CarlaUE4.sh

# Terminal 2: CARLA ROS bridge
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py

# Terminal 3: HD map visualizer
source install/setup.bash
ros2 launch hd_map hd_map_visualizer.launch.py

# Terminal 4: Mission planner
source install/setup.bash
ros2 launch mission_planner mission_planner.launch.py

# Terminal 5: Behavior planner
source install/setup.bash
ros2 launch behavior_planner behavior_planner.launch.py

# Terminal 6: Local planner
source install/setup.bash
ros2 launch local_planner local_planner.launch.py

# Terminal 7: Velocity profiler
source install/setup.bash
ros2 launch velocity_profiler velocity_profiler.launch.py

# Terminal 8: Controller
source install/setup.bash
ros2 launch controller controller.launch.py

# Terminal 9: RViz2
source install/setup.bash
rviz2 -d src/planning_stack_bringup/rviz/planning_stack.rviz
```

Wait for `[carla_ros_bridge]: Connected to CARLA` in Terminal 2 before starting the rest.

### RViz2 workflow

1. Confirm the fixed frame is set to `map`.
2. Confirm the HD map markers are visible (lane centerlines, junction spheres).
3. In the RViz2 toolbar, select the **2D Goal Pose** tool.
4. Click and drag on the map to set a goal position and heading.
5. Watch the route appear as a white path, and the vehicle begin driving toward it.

| Display                  | Topic                                | Type        |
| ------------------------ | ------------------------------------- | ----------- |
| Lane centerlines         | `/hd_map/centerlines`                 | MarkerArray |
| Lane boundaries          | `/hd_map/boundaries`                  | MarkerArray |
| Junctions                | `/hd_map/junctions`                   | MarkerArray |
| Global route             | `/mission_planner/route_display`      | Path        |
| Local trajectory         | `/local_planner/debug/trajectory`     | Path        |
| Velocity profile markers | `/velocity_profiler/debug/markers`    | MarkerArray |
| Ego vehicle odometry     | `/carla/ego_vehicle/odometry`         | Odometry    |

## Topic architecture

```
CARLA bridge
  ├──▶ /carla/ego_vehicle/odometry         (nav_msgs/Odometry)
  ├──▶ /carla/objects                      (derived_object_msgs/ObjectArray)
  ├──▶ /carla/traffic_lights/status        (carla_msgs/CarlaTrafficLightStatusList)
  └──▶ /carla/traffic_lights/info          (carla_msgs/CarlaTrafficLightInfoList)

RViz2
  └──▶ /goal_pose                          (geometry_msgs/PoseStamped)

mission_planner
  ├── sub: /carla/ego_vehicle/odometry
  ├── sub: /goal_pose
  ├── pub: /mission_planner/route          (planning_msgs/Route)  [transient_local]
  └── pub: /mission_planner/route_display  (nav_msgs/Path)        [transient_local]

behavior_planner
  ├── sub: /mission_planner/route          [transient_local]
  ├── sub: /carla/ego_vehicle/odometry
  ├── sub: /carla/objects
  ├── sub: /carla/traffic_lights/status
  ├── sub: /carla/traffic_lights/info      [transient_local]
  └── pub: /behavior_planner/behavior_command  (planning_msgs/BehaviorCommand)

local_planner
  ├── sub: /mission_planner/route          [transient_local]
  ├── sub: /carla/ego_vehicle/odometry
  ├── sub: /carla/objects
  ├── sub: /behavior_planner/behavior_command
  ├── pub: /local_planner/trajectory       (planning_msgs/Trajectory)
  └── pub: /local_planner/debug/trajectory (nav_msgs/Path)

velocity_profiler
  ├── sub: /local_planner/trajectory
  ├── sub: /carla/ego_vehicle/odometry
  ├── pub: /velocity_profiler/trajectory          (planning_msgs/Trajectory)
  ├── pub: /velocity_profiler/debug/speed_profile (std_msgs/Float32MultiArray)
  └── pub: /velocity_profiler/debug/markers       (visualization_msgs/MarkerArray)

controller
  ├── sub: /velocity_profiler/trajectory
  ├── sub: /carla/ego_vehicle/odometry
  └── pub: /carla/ego_vehicle/vehicle_control_cmd (carla_msgs/CarlaEgoVehicleControl)
```

## Custom messages (`planning_msgs`)

| Message                       | Used by                              | Description                                                                             |
| ------------------------------ | ------------------------------------- | ----------------------------------------------------------------------------------------- |
| `planning_msgs/Route`          | mission_planner → behavior_planner, local_planner | Ordered route waypoints with heading, curvature, and road-width metadata                |
| `planning_msgs/RouteWaypoint`  | Route                                 | Single waypoint with position, heading, curvature, road/lane half-widths, junction flag |
| `planning_msgs/BehaviorCommand`| behavior_planner → local_planner      | Selected behavior mode, target speed, safe distance, reason string                      |
| `planning_msgs/Trajectory`     | local_planner → velocity_profiler → controller | Ordered trajectory points with geometry, speed cap, and stop metadata                   |
| `planning_msgs/TrajectoryPoint`| Trajectory                            | Single point: pose, max velocity, profiled velocity, acceleration, timing, stop flag    |

## HD map summary (Town01)

| Item             | Value |
| ---------------- | ----: |
| Lanelets         |   124 |
| Junctions        |    12 |
| Waypoint spacing | 1.0 m |

## Key parameters

| Package | Parameter | Default | Description |
| --- | --- | --- | --- |
| behavior_planner | `normal_speed` | 8.0 m/s | Lane following speed |
| behavior_planner | `junction_detection_distance` | 12.0 m | Traffic light detection range |
| behavior_planner | `minimum_avoidance_distance` | 3.0 m | Minimum runway for avoidance |
| local_planner | `hybrid_astar.max_iterations` | 15000 | A* search depth |
| local_planner | `hybrid_astar.heading_bins` | 72 | Heading discretization |
| local_planner | `obstacle_detection.lookahead` | 30.0 m | Obstacle detection range |
| velocity_profiler | `cruise_speed` | 8.33 m/s | Nominal cruise speed |
| velocity_profiler | `a_max` | 2.0 m/s² | Maximum acceleration |
| velocity_profiler | `a_dec` | 3.0 m/s² | Maximum deceleration |
| controller | `controller.lookahead_distance` | (see YAML) | Pure Pursuit base lookahead |

## Design notes

### HD map

The map is stored as a Lanelet-inspired JSON rather than a standard format such as Lanelet2 or OpenDRIVE. That keeps the format readable, compact, and directly loadable from a custom C++ library without external parser dependencies. The PCD map is the geometric source for the road surface; CARLA's Python API annotates it semantically (lane IDs, topology, driving direction, junction information, traffic-light associations). Traffic lights are not stored as dynamic state in the map JSON. The behavior planner reads live position and state from the CARLA ROS bridge topics at runtime. The map contains 12 T-junctions and 104 lanelets that participate in closed road loops.

### Mission planner

Lanelet snapping uses a 1-metre snap radius with a 90-degree heading tolerance. If no heading-compatible lanelet is found within the radius, the planner falls back to the raw nearest-lanelet search. The A* step cost is the driven distance into the next lanelet (connection gap plus lanelet length), and the heuristic is the straight-line distance from the end of the current lanelet to the end of the goal lanelet. That heuristic never overestimates the remaining cost, so A* returns optimal lanelet sequences; a test checks this against Dijkstra. Route waypoints carry curvature, road half-width, and lane half-width metadata for downstream use.

### Behavior planner

The behavior tree uses BehaviorTree.CPP v4 with a pure Fallback (Selector) root; priority is encoded by branch order rather than explicit priority values. Condition nodes access the context through a read-only `ConditionView` (plus avoidance-latch mutators), and action nodes through a write-only `ActionView`, so cross-writes between behavior tree nodes are not possible. The BT tick loop releases the context mutex before calling `tickOnce()`; this is safe under the single-threaded `rclcpp::spin` executor the node requires, but would need re-checking under a multi-threaded executor.

### Local planner

The local planner node is split into independently testable helper classes (`ObstacleProcessor`, `AvoidanceLatch`, `StopLatch`), with the node itself as a thin ROS wrapper. The Hybrid A* grid key packs x, y, and heading each into 16 bits, supporting up to 65535 heading bins (72 by default). Obstacle avoidance uses Hybrid A* with a Dubins-path heuristic, constrained to the road corridor, with the avoidance goal set past the obstacle so the path returns to the route automatically.

### Velocity profiler

The profiler runs a forward pass (acceleration limit), zeroes speed at and after every stop point, then runs a backward braking pass over the whole profile so that any drop in the speed cap (a curve, a speed-limited zone, a stop) gets a feasible ramp. A jerk-smoothing pass then lowers speeds where the discrete jerk exceeds `j_max`, and because lowering speeds can break the other two bounds, the acceleration and braking passes run once more. The output therefore satisfies `a_max` and `a_dec` between every pair of points; `j_max` is best effort. Stop points come from the `is_stop_point` flag set by the local planner.

### Controller

Pure Pursuit uses a speed-scaled dynamic lookahead: `Ld = max(min_Ld, base_Ld + gain × speed)`, improving lateral tracking stability at higher speed while preserving tight tracking at low speed. CARLA uses a left-positive steering convention opposite to the ROS mathematical convention, so the controller inverts sign before publishing (`command.steer = -filtered_steer`). Stale trajectory/odometry detection uses wall-clock reception time rather than message timestamps, so the controller stays safe during rosbag replay with non-monotonic simulation timestamps.

## Tests

70 gtest cases, run with `colcon test` (no simulator needed):

| Package | Tests | What they check |
| --- | ---: | --- |
| `hd_map` | 24 | Map loading, lanelet and junction queries, successor/predecessor topology, ROS-frame geometry |
| `mission_planner` | 16 | Heading-aware lane snapping, route content and trimming, and A* optimality: on 110 start/goal pairs across Town01 the route cost matches an independent Dijkstra search |
| `local_planner` | 8 | Hybrid A* finds paths in open space and around an obstacle inside the road corridor, every step respects the minimum turning radius, and blocked or invalid goals fail cleanly; cubic spline keeps endpoints, spacing, straight lines and circle curvature |
| `velocity_profiler` | 9 | Acceleration and braking limits hold between every pair of points (including curve entry and speed-limited zones), stops and caps are respected, and invalid parameters are rejected |
| `controller` | 13 | Pure Pursuit steering sign and geometry (a circle of radius R gives `atan(L / R)`), saturation, speed-command clamping, vehicle-model limits, and two closed-loop runs on the kinematic model |

The closed-loop controller tests drive the kinematic bicycle model with the tuned YAML gains, a 20 Hz loop and the steering-rate limit:

| Scenario (5 m/s) | Max cross-track error | RMS |
| --- | ---: | ---: |
| Straight line, starting 1.5 m off the path (after 15 s) | < 0.001 m | < 0.001 m |
| Circle, R = 20 m (after 10 s) | 0.25 m | 0.25 m |

The 0.25 m offset on the circle is the usual Pure Pursuit corner-cutting at this lookahead. These numbers come from the kinematic model, not from CARLA's vehicle dynamics. `behavior_planner/tests/` holds two helper scripts that publish fake obstacles for manual testing in CARLA.

## Known limits

- The HD map format is a custom JSON, not Lanelet2 or OpenDRIVE, so it only works with this stack's own loader.
- Only Town01 has been built and exercised; other towns would need their own point cloud and a re-run of the map generator.
- The controller implements Pure Pursuit only; Stanley and optimization-based controllers (MPC/MPPI) are not implemented.
- The controller steers the front wheels only. The vehicle model has a counter-phase rear-steering mode, but CARLA's control message has a single steering field, so the node refuses to start with it enabled.
- `behavior_planner` has no unit tests; it is exercised in CARLA with the fake-obstacle scripts.
- Closed-loop numbers in [Tests](#tests) come from a kinematic model, not from CARLA runs.
- The behavior planner's BT tick loop assumes a single-threaded executor (see [Design notes](#design-notes)).
