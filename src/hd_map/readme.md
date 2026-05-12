# HD Map Generation

## Overview

This package implements a lightweight HD map pipeline for CARLA Town01.

It generates a Lanelet-inspired JSON map using:

- CARLA semantic road data
- Town01 PCD ground geometry

The generated map is loaded by a C++ runtime library and used by the downstream planning modules.

Generated map:

`maps/Town01_hd_map.json`

Current generated map summary:

| Item             | Value |
| ---------------- | ----: |
| Lanelets         |   124 |
| Junctions        |    12 |
| Waypoint spacing | 1.0 m |

## Purpose

The goal of this task is to provide a planning-oriented HD map representation that supports:

- lane-level routing
- graph-based mission planning
- junction awareness
- lane boundary visualization
- downstream behavior and local planning

This is not intended to replace Lanelet2 or OpenDRIVE. It is a compact map format designed to be simple to parse and query from C++.

## System Design

The package is divided into three parts:

1. Python map generation
2. Lanelet-inspired JSON map
3. C++ runtime loader and query API

Pipeline:

```text
CARLA + Town01 PCD
        ↓
Python Map Generator
        ↓
Town01_hd_map.json
        ↓
C++ HDMap Library
        ↓
Planning Modules / RViz Visualization
```

## Python map generation

The Python pipeline:

1. Loads the Town01 PCD file.
2. Filters ground points from the point cloud.
3. Builds a KD-tree for ground height lookup.
4. Connects to CARLA.
5. Extracts lane centerlines, topology, and junctions.
6. Builds lane boundaries using lane width and heading.
7. Saves the generated map as JSON.

Main files:

```text
generate_hd_map.py
map_generator/pcd_processor.py
map_generator/carla_extractor.py
map_generator/map_builder.py
```

## C++ Runtime Map Library

The C++ library loads the generated JSON map and provides query functions for planning modules.

Main files:

```text
include/hd_map/hd_map.hpp
src/hd_map.cpp
```

Supported queries:

- get all lanelets
- get lanelet by ID
- get nearest lanelet to a position
- get lanelet successors
- get lanelet predecessors
- get all junctions
- get junction by ID
- check whether a lanelet is inside a junction

## ROS 2 Visualization

The package includes a ROS 2 visualization node that publishes the map as RViz markers.

Main files:

```text
include/hd_map/hd_map_visualizer_node.hpp
src/hd_map_visualizer_node.cpp
launch/hd_map_visualizer.launch.py
```

Published marker topics:

```text
/hd_map/centerlines
/hd_map/boundaries
/hd_map/junctions
/hd_map/lane_ids
/hd_map/directions
```

The publishers use transient local QoS so RViz can still receive the markers if it starts after the node.

## Map Representation

The map is stored as JSON with three main sections:

```json
{
  "metadata": {},
  "lanelets": [],
  "junctions": []
}
```

## Lanelet Structure

Each lanelet represents one directed drivable lane segment.

Example:

```json
{
  "id": "lanelet_1_-1",
  "road_id": 1,
  "lane_id": -1,
  "is_junction": false,
  "junction_id": -1,
  "length_m": 156.0,
  "centerline": [
    {
      "x": 0.0,
      "y": 0.0,
      "z": 0.0,
      "heading_rad": 0.0
    }
  ],
  "left_boundary": [
    {
      "x": 0.0,
      "y": 2.0,
      "z": 0.0
    }
  ],
  "right_boundary": [
    {
      "x": 0.0,
      "y": -2.0,
      "z": 0.0
    }
  ],
  "predecessors": [],
  "successors": [],
  "left_neighbor": "",
  "right_neighbor": ""
}
```

## Lane Graph

Lane connectivity is represented using:

```text
predecessors
successors
```

For example:

```text
lanelet_A → lanelet_B → lanelet_C
```

For `lanelet_B`:

```text
predecessor: lanelet_A
successor:   lanelet_C
```

This allows the mission planner to use the HD map as a directed graph.

## Junction Representation

Junctions are extracted from CARLA and stored separately.

Example:

```json
{
  "id": 26,
  "center_x": 337.0804,
  "center_y": -2.0453,
  "type": "T-junction",
  "connecting_roads": [0, 1, 16],
  "internal_lane_ids": [
    "lanelet_27_1",
    "lanelet_33_1"
  ]
}
```

Lanelets inside junctions are marked with:

```json
"is_junction": true
```

and linked to the junction using:

```json
"junction_id": 26
```

## Package Structure

```text
hd_map/
├── CMakeLists.txt
├── package.xml
├── generate_hd_map.py
├── map_generator/
│   ├── __init__.py
│   ├── pcd_processor.py
│   ├── carla_extractor.py
│   └── map_builder.py
├── include/hd_map/
│   ├── hd_map.hpp
│   └── hd_map_visualizer_node.hpp
├── src/
│   ├── hd_map.cpp
│   └── hd_map_visualizer_node.cpp
├── launch/
│   └── hd_map_visualizer.launch.py
├── maps/
│   └── Town01_hd_map.json
├── scripts/
│   ├── analyze_pcd_content.py
│   └── explore_map.py
└── tests/
    ├── test_hd_map_node.cpp
    ├── test_pcd_processor.py
    ├── test_carla_extractor.py
    └── test_map_builder.py
```

## Dependencies

Tested environment:

```text
Ubuntu 22.04
ROS 2 Humble
CARLA 0.9.16
Python 3.10+
```

ROS dependencies:

```text
rclcpp
visualization_msgs
geometry_msgs
std_msgs
ament_cmake
ament_cmake_gtest
ament_index_cpp
```

Python dependencies:

```bash
pip3 install numpy scipy open3d
```

The CARLA Python API must also be available in the Python environment.

## Build

From the workspace root:

```bash
colcon build --packages-select hd_map
source install/setup.bash
```

## Generate the HD Map

Start CARLA first:

```bash
cd ~/CARLA_0.9.16
./CarlaUE4.sh
```

Make sure the Town01 PCD file exists at:

```text
data/Town01.pcd
```

Then run:

```bash
cd planning_stack/src/hd_map
python3 generate_hd_map.py Town01
```

Generated output:

```text
maps/Town01_hd_map.json
```

## Run the Visualizer

After building the package:

```bash
ros2 launch hd_map hd_map_visualizer.launch.py
```

Or pass a custom map path:

```bash
ros2 launch hd_map hd_map_visualizer.launch.py map_path:=/absolute/path/to/Town01_hd_map.json
```

In RViz, use:

```text
Fixed Frame: map
```

Then add the marker topics:

```text
/hd_map/centerlines
/hd_map/boundaries
/hd_map/junctions
/hd_map/lane_ids
/hd_map/directions
```

## C++ API Usage

Include the map API:

```cpp
#include "hd_map/hd_map.hpp"
```

Load the map:

```cpp
hd_map::HDMap map;

if (!map.load("maps/Town01_hd_map.json")) {
  return;
}
```

Get a lanelet by ID:

```cpp
const hd_map::Lanelet * lane =
  map.getLaneletById("lanelet_1_-1");
```

Find nearest lanelet:

```cpp
const hd_map::Lanelet * nearest =
  map.getNearestLanelet(x, y);
```

Get successors:

```cpp
auto successors =
  map.getSuccessors("lanelet_1_-1");
```

Get predecessors:

```cpp
auto predecessors =
  map.getPredecessors("lanelet_1_-1");
```

Check junction membership:

```cpp
bool in_junction =
  map.isInJunction("lanelet_27_1");
```

Get junction by ID:

```cpp
const hd_map::Junction * junction =
  map.getJunctionById(26);
```

## Tests

Run C++ tests:

```bash
colcon test --packages-select hd_map
colcon test-result --verbose
```

The C++ tests validate:

- map loading
- metadata parsing
- lanelet lookup
- nearest lanelet query
- successor and predecessor queries
- junction lookup
- junction classification

Python test files are also provided for the generation pipeline:

```bash
python3 tests/test_pcd_processor.py
python3 tests/test_carla_extractor.py
python3 tests/test_map_builder.py
```

Some Python tests require CARLA to be running.
