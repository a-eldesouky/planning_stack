import sys
import os
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from map_generator.carla_extractor import CarlaExtractor


def assert_normalized_angle(angle: float):
    assert -math.pi <= angle <= math.pi, \
        f"Angle is not normalized to [-pi, pi]: {angle}"


def test_connect():
    """Test CARLA connection, map loading, and data caching."""
    print("\n=== Test 1: Connect to CARLA ===")

    extractor = CarlaExtractor(map_name='Town01', waypoint_spacing=2.0)
    ok = extractor.connect()

    assert ok, "Connection failed"
    assert extractor.world is not None, "World is None"
    assert extractor.carla_map is not None, "Map is None"
    assert 'Town01' in extractor.carla_map.name, "Wrong map loaded"

    # Verify cached data was populated in connect()
    assert extractor._topology  is not None, "Topology not cached"
    assert extractor._waypoints is not None, "Waypoints not cached"
    assert len(extractor._topology)  == 160, \
        f"Expected 160 topology edges, got {len(extractor._topology)}"
    assert len(extractor._waypoints) > 3000, \
        f"Expected >3000 waypoints, got {len(extractor._waypoints)}"

    print(f"  Map name          : {extractor.carla_map.name}")
    print(f"  Waypoint spacing  : {extractor.waypoint_spacing}m")
    print(f"  Topology edges    : {len(extractor._topology)}")
    print(f"  Cached waypoints  : {len(extractor._waypoints)}")
    print("  PASSED")
    return extractor


def test_custom_spacing():
    """Test that waypoint spacing is respected in centerline extraction."""
    print("\n=== Test 2: Custom Waypoint Spacing ===")

    extractor_1m = CarlaExtractor(map_name='Town01', waypoint_spacing=1.0)
    ok           = extractor_1m.connect()
    assert ok, "Connection failed for 1m spacing"

    centerlines_1m = extractor_1m.get_centerlines()
    sample_1m = centerlines_1m[(1, -1)]

    extractor_2m = CarlaExtractor(map_name='Town01', waypoint_spacing=2.0)
    ok = extractor_2m.connect()
    assert ok, "Connection failed for 2m spacing"

    centerlines_2m = extractor_2m.get_centerlines()
    sample_2m = centerlines_2m[(1, -1)]

    assert len(sample_1m) > len(sample_2m), \
        "1m spacing should give more points than 2m"

    print(f"  Road 1 Lane -1 at 1m spacing : {len(sample_1m)} points")
    print(f"  Road 1 Lane -1 at 2m spacing : {len(sample_2m)} points")
    print("  PASSED")
    return extractor_2m


def test_get_centerlines(extractor):
    """Test centerline extraction in ROS map frame."""
    print("\n=== Test 3: Get Centerlines in ROS Frame ===")
    centerlines = extractor.get_centerlines()

    assert len(centerlines) == 124, \
        f"Expected 124 centerlines, got {len(centerlines)}"

    assert (1, -1) in centerlines, "Lane (road=1, lane=-1) missing"

    sample = centerlines[(1, -1)]
    assert len(sample) > 0, "Empty centerline for road 1 lane -1"

    required_fields = {
        "x", "y", "z", "heading_rad",
        "lane_width", "is_junction", "junction_id"
    }
    for field in required_fields:
        assert field in sample[0], f"Missing field: {field}"

    first = sample[0]
    assert abs(first["y"] - 2.0) < 1.0, \
        f"Expected road 1 lane -1 near ROS y≈+2.0, got {first['y']}"

    assert_normalized_angle(first["heading_rad"])

    assert (19, 1) in centerlines, "Lane (road=19, lane=1) missing"
    lane_19_1 = centerlines[(19, 1)]
    nearest_to_ego = min(
        lane_19_1,
        key=lambda p: (p["x"] - 339.0) ** 2 + (p["y"] + 314.0) ** 2
    )
    dist = math.hypot(nearest_to_ego["x"] - 339.0,
                      nearest_to_ego["y"] + 314.0)
    assert dist < 3.0, \
        f"Expected lane (19,1) near ROS ego pose (339,-314), got {nearest_to_ego}, dist={dist:.2f}"
    assert nearest_to_ego["y"] < 0.0, \
        "Expected lane near ego to have negative ROS Y"
    assert_normalized_angle(nearest_to_ego["heading_rad"])

    print(f"  Total centerlines     : {len(centerlines)}")
    print(f"  Road 1 Lane -1 points : {len(sample)}")
    print(f"  Road 1 Lane -1 first  : {first}")
    print(f"  Road 19 Lane 1 ego-nearest point: {nearest_to_ego}")
    print("  PASSED")
    return centerlines


def test_get_lane_graph(extractor):
    """Test lane graph connectivity."""
    print("\n=== Test 4: Get Lane Graph ===")
    graph = extractor.get_lane_graph()

    assert 'predecessors' in graph, "Missing predecessors"
    assert 'successors'   in graph, "Missing successors"

    lane_id = "lanelet_1_-1"
    assert lane_id in graph['successors'], \
        f"{lane_id} not in successors"
    assert len(graph['successors'][lane_id]) == 2, \
        f"Expected 2 successors for {lane_id}, " \
        f"got {len(graph['successors'][lane_id])}"

    lane_25 = "lanelet_25_-1"
    assert lane_25 in graph['successors'], \
        f"{lane_25} not in successors"

    print(f"  Total lanes with successors  : {len(graph['successors'])}")
    print(f"  Total lanes with predecessors: {len(graph['predecessors'])}")
    print(f"  Successors of lanelet_1_-1   : {graph['successors'][lane_id]}")
    print(f"  Successors of lanelet_25_-1  : {graph['successors'][lane_25]}")
    print("  PASSED")
    return graph


def test_get_junctions(extractor):
    """Test junction extraction: expect 12 T-junctions."""
    print("\n=== Test 5: Get Junctions ===")
    junctions = extractor.get_junctions()

    assert len(junctions) == 12, \
        f"Expected 12 junctions, got {len(junctions)}"

    # Find junction 332
    j332 = next((j for j in junctions if j['id'] == 332), None)
    assert j332 is not None,                      "Junction 332 not found"
    assert j332['type'] == 'T-junction',          "Wrong type for j332"
    assert sorted(j332['connecting_roads']) == [9, 10, 25], \
        f"Wrong connecting roads: {j332['connecting_roads']}"

    assert j332['center_y'] < 0.0, \
        f"Expected junction 332 center_y to be negative in ROS frame, got {j332['center_y']}"

    j54 = next((j for j in junctions if j['id'] == 54), None)
    assert j54 is not None, "Junction 54 not found"
    assert j54['type'] == 'T-junction', "Wrong type for j54"

    for j in junctions:
        assert j['type'] == 'T-junction', \
            f"Junction {j['id']} is not T-junction: {j['type']}"

    print(f"  Total junctions        : {len(junctions)}")
    print(f"  Junction 332 center    : ({j332['center_x']:.2f}, {j332['center_y']:.2f})")
    print(f"  Junction 332 roads     : {j332['connecting_roads']}")
    print(f"  Junction 54  center    : ({j54['center_x']:.2f}, {j54['center_y']:.2f})")
    print("  PASSED")
    return junctions


def main():
    extractor = test_connect()
    test_custom_spacing()

    test_get_centerlines(extractor)
    test_get_lane_graph(extractor)
    test_get_junctions(extractor)

    print("\n=== All tests passed ===\n")


if __name__ == '__main__':
    main()
