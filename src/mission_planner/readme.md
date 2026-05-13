# Mission Planner

## Overview

This package implements the mission planning layer for the autonomous driving planning stack.

It generates a global route from the current ego vehicle position to a goal pose selected in RViz2.

The mission planner uses:

* CARLA ego vehicle odometry
* RViz2 goal pose
* Pre-generated HD map JSON
* A* search over the lanelet graph

The output route is published for downstream planning modules and also visualized in RViz2.

Published outputs:

```text
/mission_planner/route
/mission_planner/route_display
```

## Purpose

The goal of this task is to provide a global route that can be used by:

* Behavior Planner
* Local Planner
* Velocity Profile Generator
* Controller

The mission planner is responsible for high-level routing only.

It does not handle:

* obstacle avoidance
* lane-change decision making
* velocity optimization
* trajectory tracking

Those responsibilities are handled by later planning stack modules.

## System Design

The package is divided into two main parts:

1. Pure C++ mission planner library
2. ROS 2 mission planner node

Pipeline:

```text
CARLA Odometry + RViz Goal
        ↓
MissionPlannerNode
        ↓
MissionPlanner Library
        ↓
HD Map Lanelet Graph
        ↓
A* Route Search
        ↓
planning_msgs/Route + nav_msgs/Path
```

## Mission planner library

The core planner is implemented as a pure C++ class.

Main files:

```text
include/mission_planner/mission_planner.hpp
src/mission_planner.cpp
```

Responsibilities:

* load the HD map
* find the nearest lanelet to the ego pose
* find the nearest lanelet to the goal pose
* consider vehicle heading during lanelet snapping
* run A* over lanelet successors
* build a route from the selected lanelet sequence
* trim the start and goal lanelets based on the actual poses

This keeps the planning logic independent from ROS 2 communication.

## Mission planner node

The ROS 2 node wraps the planner library.

Main files:

```text
include/mission_planner/mission_planner_node.hpp
src/mission_planner_node.cpp
```

Responsibilities:

* subscribe to ego odometry
* subscribe to goal pose
* call the mission planner
* convert the route to `planning_msgs/Route`
* publish a display path as `nav_msgs/Path`

Subscribed topics:

```text
/carla/ego_vehicle/odometry
/goal_pose
```

Published topics:

```text
/mission_planner/route
/mission_planner/route_display
```

## Planning Approach

The mission planner treats the HD map as a directed graph.

```text
Node      = lanelet
Edge      = successor relationship
Cost      = connection gap + length of the lanelet entered
Heuristic = straight line from the end of the lanelet to the end of the goal lanelet
```

A* is used to search from the start lanelet to the goal lanelet. The heuristic never overestimates the remaining driven distance, so the search returns an optimal lanelet sequence. `tests/test_mission_planner.cpp` checks this against Dijkstra on 110 start/goal pairs.

Planning flow:

```text
Ego pose
   ↓
Find nearest valid start lanelet
   ↓
Goal pose
   ↓
Find nearest valid goal lanelet
   ↓
Run A* on lanelet graph
   ↓
Build route waypoints
   ↓
Publish route
```

## Heading-Aware Lanelet Snapping

The planner does not use position-only snapping.

It also checks the heading difference between the vehicle pose and lanelet heading.

This is important because opposite-direction lanes may be close to each other.

Example:

```text
Same road area
    ↓
north heading → north lane
south heading → south lane
```

This prevents the planner from selecting the wrong lane direction.

## A* Search

The A* search operates on lanelet IDs.

Each lanelet expands to its successor lanelets from the HD map.

The planner stores:

* `g_cost`: accumulated route cost
* `came_from`: parent relation for path reconstruction
* `closed`: visited lanelets
* priority queue: open set ordered by total estimated cost

The result is an ordered list of lanelet IDs.

Example:

```text
lanelet_25_-1
      ↓
lanelet_333_-1
      ↓
lanelet_9_1
```

## Route Trimming

A* returns full lanelets, but the vehicle may start or stop in the middle of a lanelet.

Therefore, the route builder trims:

* the first lanelet from the closest point to the ego pose
* the last lanelet up to the closest point to the goal pose
* middle lanelets are used fully

This avoids generating a route that starts behind the vehicle or continues beyond the goal.

## Route Message

The package publishes a custom route message:

```text
planning_msgs/msg/Route
```

The route contains:

* header
* route waypoints
* total route length
* start lanelet ID
* goal lanelet ID

Each route waypoint includes useful metadata for later modules:

* position
* heading
* road half width
* lane half width
* junction flag
* curvature estimate

This makes the route useful for planning, not only visualization.

## Curvature Estimation

The node estimates curvature for route waypoints using a three-point geometric method.

```text
previous waypoint
current waypoint
next waypoint
```

This provides a simple curvature estimate that can be used later by the velocity profile generator.

Sharp curves can be assigned lower speeds in later tasks.

## Lane width metadata

For each waypoint, the node estimates lane and road width information using the nearest lanelet boundaries.

These fields are useful for:

* normal lane following
* local planning constraints
* future obstacle avoidance logic

If boundaries are not available, default values are used instead of failing the route.

## ROS 2 Interfaces

### Subscribed Topics

| Topic                           | Type                              | Description              |
| ------------------------------- | --------------------------------- | ------------------------ |
| `/carla/ego_vehicle/odometry` | `nav_msgs/msg/Odometry`         | Current ego vehicle pose |
| `/goal_pose`                  | `geometry_msgs/msg/PoseStamped` | Goal pose from RViz2     |

### Published Topics

| Topic                              | Type                        | Description                           |
| ---------------------------------- | --------------------------- | ------------------------------------- |
| `/mission_planner/route`         | `planning_msgs/msg/Route` | Route for downstream planning modules |
| `/mission_planner/route_display` | `nav_msgs/msg/Path`       | Route visualization in RViz2          |

## Package Structure

```text
mission_planner/
├── CMakeLists.txt
├── package.xml
├── include/mission_planner/
│   ├── mission_planner.hpp
│   └── mission_planner_node.hpp
├── src/
│   ├── mission_planner.cpp
│   └── mission_planner_node.cpp
├── launch/
│   └── mission_planner.launch.py
└── tests/
    ├── test_mission_planner.cpp
    └── data/
        └── Town01_hd_map.json
```

## Dependencies

Tested environment:

```text
Ubuntu 22.04
ROS 2 Humble
CARLA 0.9.16
C++17
```

ROS dependencies:

```text
rclcpp
nav_msgs
geometry_msgs
planning_msgs
hd_map
ament_cmake
ament_cmake_gtest
```

## Build

From the workspace root:

```bash
colcon build --packages-up-to mission_planner
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

Launch the mission planner:

```bash
ros2 launch mission_planner mission_planner.launch.py
```

Or pass a custom HD map path:

```bash
ros2 launch mission_planner mission_planner.launch.py \
  hd_map_path:=/absolute/path/to/Town01_hd_map.json
```

## RViz2 Workflow

1. Open RViz2.
2. Set the fixed frame to:

```text
map
```

3. Use the `2D Goal Pose` tool.
4. Click and drag to set the target position and heading.
5. The mission planner publishes the route.

Useful topics to visualize:

```text
/mission_planner/route_display
```

## Tests

Run tests:

```bash
colcon test --packages-select mission_planner
colcon test-result --verbose
```

The tests validate:

* map loading
* invalid map path handling
* heading-aware lane snapping
* opposite-lane selection
* route generation
* route lanelet order
* junction lanelet detection
* non-empty route waypoints
* route length calculation
* start and goal trimming
* same-lane planning
* long route planning
* waypoint heading validity
* multi-hop routing
