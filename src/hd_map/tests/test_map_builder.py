import sys
import os
import json
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from map_generator.pcd_processor   import PCDProcessor
from map_generator.carla_extractor import CarlaExtractor
from map_generator.map_builder     import MapBuilder


PCD_PATH    = '../data/Town01.pcd'
OUTPUT_PATH = 'test_hd_map.json'


def assert_normalized_angle(angle: float):
    assert -math.pi <= angle <= math.pi, \
        f"Angle is not normalized to [-pi, pi]: {angle}"


def setup():
    """Initialize PCDProcessor and CarlaExtractor before tests."""
    print("\n=== Setup ===")

    # PCD
    pcd = PCDProcessor()
    ok  = pcd.load_and_filter(PCD_PATH)
    assert ok, "PCDProcessor failed to load PCD"
    ok = pcd.build_kdtree()
    assert ok, "PCDProcessor failed to build KD-tree"

    # CARLA
    carla = CarlaExtractor()
    ok    = carla.connect()
    assert ok, "CarlaExtractor failed to connect"

    print("  Setup PASSED")
    return pcd, carla


def test_build(pcd, carla):
    """Test that MapBuilder.build() returns a complete map dict."""
    print("\n=== Test 1: Build ===")
    builder  = MapBuilder(pcd, carla)
    map_dict = builder.build()

    assert isinstance(map_dict, dict), "build() did not return a dict"

    required_keys = {"metadata", "lanelets", "junctions"}
    for key in required_keys:
        assert key in map_dict, f"Missing key: {key}"

    print(f"  Map keys      : {list(map_dict.keys())}")
    print("  PASSED")
    return builder, map_dict


def test_metadata(map_dict):
    """Test metadata block has correct fields and values."""
    print("\n=== Test 2: Metadata ===")
    meta = map_dict['metadata']

    assert 'Town01'              in meta['map_name']
    assert meta['version']       == '1.0'
    assert meta['heading_unit']  == 'radians'
    assert meta['total_lanelets']       == 124
    assert meta['total_junctions']      == 12

    print(f"  Map name      : {meta['map_name']}")
    print(f"  Total lanelets: {meta['total_lanelets']}")
    print(f"  Total junctions     : {meta['total_junctions']}")
    print("  PASSED")


def test_lanelets(map_dict):
    """Test lanelet count and structure of a known lanelet."""
    print("\n=== Test 3: Lanelets ===")
    lanelets = map_dict['lanelets']

    assert len(lanelets) == 124, \
        f"Expected 124 lanelets, got {len(lanelets)}"

    # Find lanelet_1_-1
    l1 = next((l for l in lanelets if l['id'] == 'lanelet_1_-1'), None)
    assert l1 is not None, "lanelet_1_-1 not found"

    # Check required fields
    required_fields = {
        "id", "road_id", "lane_id", "is_junction", "junction_id",
        "length_m", "centerline", "left_boundary", "right_boundary",
        "predecessors", "successors", "left_neighbor", "right_neighbor"
    }
    for field in required_fields:
        assert field in l1, f"lanelet_1_-1 missing field: {field}"

    # Centerline must have points
    assert len(l1['centerline']) > 0, "Empty centerline"

    # Boundaries must match centerline length
    assert len(l1['left_boundary'])  == len(l1['centerline']), \
        "Left boundary length mismatch"
    assert len(l1['right_boundary']) == len(l1['centerline']), \
        "Right boundary length mismatch"

    # Each centerline point must have x, y, z, heading_rad
    for pt in l1['centerline']:
        for field in ('x', 'y', 'z', 'heading_rad'):
            assert field in pt, f"Centerline point missing: {field}"
        assert_normalized_angle(pt['heading_rad'])

    first_pt = l1['centerline'][0]
    assert abs(first_pt['y'] - 2.0) < 1.0, \
        f"Expected lanelet_1_-1 near ROS y≈+2.0, got {first_pt['y']}"

    # Successors
    assert sorted(l1['successors']) == ['lanelet_62_-1', 'lanelet_67_-1'], \
        f"Wrong successors: {l1['successors']}"

    # Length should be positive
    assert l1['length_m'] > 0, "Length must be positive"

    print(f"  Total lanelets          : {len(lanelets)}")
    print(f"  lanelet_1_-1 first point: {first_pt}")
    print(f"  lanelet_1_-1 centerline : {len(l1['centerline'])} points")
    print(f"  lanelet_1_-1 length     : {l1['length_m']} m")
    print(f"  lanelet_1_-1 successors : {l1['successors']}")
    print(f"  lanelet_1_-1 left_neighbor  : {l1['left_neighbor']}")
    print(f"  lanelet_1_-1 right_neighbor : {l1['right_neighbor']}")
    print("  PASSED")


def test_lanelet_near_ros_ego_pose(map_dict):
    """Confirm the generated map aligns directly with CARLA ROS odometry/RViz."""
    print("\n=== Test 4: Lanelet Near ROS Ego Pose ===")

    lanelets = map_dict['lanelets']
    lane_19_1 = next((l for l in lanelets if l['id'] == 'lanelet_19_1'), None)
    assert lane_19_1 is not None, "lanelet_19_1 not found"

    nearest = min(
        lane_19_1['centerline'],
        key=lambda p: (p['x'] - 339.0) ** 2 + (p['y'] + 314.0) ** 2
    )
    dist = math.hypot(nearest['x'] - 339.0, nearest['y'] + 314.0)

    assert dist < 3.0, \
        f"Expected lanelet_19_1 near ROS odometry sample (339,-314), got {nearest}, dist={dist:.2f}"
    assert nearest['y'] < 0.0, "Expected negative ROS Y near ego pose"
    assert_normalized_angle(nearest['heading_rad'])

    print(f"  Nearest lanelet_19_1 point to ROS ego (339,-314): {nearest}")
    print(f"  Distance: {dist:.2f} m")
    print("  PASSED")


def test_junctions_are_ros_frame(map_dict):
    """Confirm junction centers are exported in ROS frame."""
    print("\n=== Test 5: Junctions in ROS Frame ===")

    junctions = map_dict['junctions']
    assert len(junctions) == 12, \
        f"Expected 12 junctions, got {len(junctions)}"

    j332 = next((j for j in junctions if j['id'] == 332), None)
    assert j332 is not None, "Junction 332 not found"
    assert j332['type'] == 'T-junction'
    assert sorted(j332['connecting_roads']) == [9, 10, 25]
    assert j332['center_y'] < 0.0, \
        f"Expected junction 332 center_y to be negative in ROS frame, got {j332['center_y']}"

    print(f"  Junction 332 center: ({j332['center_x']:.2f}, {j332['center_y']:.2f})")
    print(f"  Junction 332 roads : {j332['connecting_roads']}")
    print("  PASSED")


def test_save(builder):
    """Test that save() writes a valid JSON file."""
    print("\n=== Test 6: Save ===")
    builder.save(OUTPUT_PATH)

    assert os.path.exists(OUTPUT_PATH), "Output file not created"

    # Reload and verify it parses correctly
    with open(OUTPUT_PATH) as f:
        reloaded = json.load(f)

    assert 'metadata'       in reloaded
    assert 'lanelets'       in reloaded
    assert 'junctions'      in reloaded
    assert len(reloaded['lanelets']) == 124

    size_mb = os.path.getsize(OUTPUT_PATH) / (1024 * 1024)
    print(f"  File saved    : {OUTPUT_PATH}")
    print(f"  File size     : {size_mb:.2f} MB")
    print(f"  Lanelets      : {len(reloaded['lanelets'])}")
    print("  PASSED")


def main():
    pcd, carla = setup()
    builder, map_dict = test_build(pcd, carla)
    test_metadata(map_dict)
    test_lanelets(map_dict)
    test_lanelet_near_ros_ego_pose(map_dict)
    test_junctions_are_ros_frame(map_dict)
    test_save(builder)

    print("\n=== All tests passed ===\n")


if __name__ == '__main__':
    main()
