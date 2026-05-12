import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from map_generator.pcd_processor import PCDProcessor

def test_load_and_filter(pcd_path):
    """Test loading and ground filtering."""
    print("\n=== Test 1: Load and Filter ===")
    pcd = PCDProcessor()
    ok  = pcd.load_and_filter(pcd_path)
    assert ok, "load_and_filter failed"
    assert pcd.ground_points is not None
    assert pcd.ground_points.shape[1] == 3
    print(f"  PASSED: {len(pcd.ground_points)} ground points")
    return pcd

def test_kdtree(pcd):
    """Test KD-tree builds successfully."""
    print("\n=== Test 2: Build KD-tree ===")
    ok = pcd.build_kdtree()
    assert ok, "build_kdtree failed"
    assert pcd.kdtree is not None
    print("  PASSED: KD-tree built")

def test_get_z_at(pcd):
    """Test Z lookup at known Town01 road positions in ROS map frame."""
    print("\n=== Test 3: Z lookup at known ROS-frame road positions ===")

    # These are known CARLA waypoint positions converted to ROS/RViz frame:
    # x_ros = x_carla, y_ros = -y_carla.
    known_positions = [
        (144.99, -59.49, "junction 332 area"),
        (168.08,  -1.96, "road 1 start"),
        (325.63,   2.01, "road 1 end"),
        (167.17, -59.49, "road 10 start"),
        (325.16, -55.47, "road 10 end"),
        (339.00, -314.0, "near ROS ego odometry sample"),
    ]

    for x, y, label in known_positions:
        z = pcd.get_z_at(x, y)
        print(f"  ({x:>8.2f}, {y:>8.2f})  z={z:>7.4f}  [{label}]")
        # Z can genuinely be 0.0 on flat road, so this test is mainly a smoke
        # check that ROS-frame queries do not throw and return a float.
        assert isinstance(z, float), f"Z lookup did not return float for ({x}, {y})"

    print("  PASSED")

def test_boundary_extraction(pcd):
    """Test lane boundary extraction on a sample centerline."""
    print("\n=== Test 4: Lane boundary extraction ===")

    # Sample centerline from road 1 lane -1 (horizontal road, heading ≈ 0)
    centerline = [
        {"x": 170.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 172.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 174.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 176.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 178.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
    ]
    lane_width = 4.0

    left  = pcd.extract_lane_boundary(centerline, lane_width, 'left')
    right = pcd.extract_lane_boundary(centerline, lane_width, 'right')

    assert len(left)  == len(centerline), "Left boundary length mismatch"
    assert len(right) == len(centerline), "Right boundary length mismatch"

    print(f"  Centerline points : {len(centerline)}")
    print(f"  Left  boundary    : {len(left)} points")
    print(f"  Right boundary    : {len(right)} points")

    # Because boundaries are computed directly in ROS frame, the usual left/right
    # convention remains valid. For heading=0, left is y+2 and right is y-2.
    for i, (cl, lb, rb) in enumerate(zip(centerline, left, right)):
        expected_left_y = cl['y'] + 2.0
        expected_right_y = cl['y'] - 2.0
        print(f"  Point {i}: center_y={cl['y']:.2f} "
              f"left_y={lb['y']:.4f} (expected≈{expected_left_y:.2f})  "
              f"right_y={rb['y']:.4f} (expected≈{expected_right_y:.2f})")
        assert abs(lb['y'] - expected_left_y) < 1e-3, \
            f"Left boundary Y mismatch at point {i}"
        assert abs(rb['y'] - expected_right_y) < 1e-3, \
            f"Right boundary Y mismatch at point {i}"

    print("  PASSED")
    return centerline, left, right

def test_z_enrichment(pcd):
    """Test that centerline Z values get replaced with PCD heights."""
    print("\n=== Test 5: Z enrichment ===")

    centerline = [
        {"x": 200.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 210.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
        {"x": 220.0, "y": -1.96, "z": 0.0, "heading_rad": -0.0},
    ]

    enriched = pcd.enrich_centerline_z(centerline)
    print("  Before: all z=0.0")
    for pt in enriched:
        print(f"  ({pt['x']}, {pt['y']}) → z={pt['z']}")
        assert isinstance(pt['z'], float), "Enriched z must be a float"

    print("  PASSED")

def visualize(pcd, centerline, left, right):
    """Plot ground points + sample lane boundaries to verify geometry."""
    print("\n=== Visualization ===")

    # Sample ground points for plotting
    pts    = pcd.ground_points
    sample = pts[::100]   # every 100th point

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_title("PCDProcessor: Ground Points + Sample Lane Boundaries")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal')
    ax.grid(True, linestyle='--', alpha=0.3)

    ax.scatter(sample[:, 0], sample[:, 1],
               s=0.5, color='lightgray', label='Ground points (sampled)')

    # Centerline
    cx = [p['x'] for p in centerline]
    cy = [p['y'] for p in centerline]
    ax.plot(cx, cy, 'b-o', linewidth=2, markersize=5, label='Centerline')

    # Left boundary
    lx = [p['x'] for p in left]
    ly = [p['y'] for p in left]
    ax.plot(lx, ly, 'g--o', linewidth=1.5, markersize=4, label='Left boundary')

    # Right boundary
    rx = [p['x'] for p in right]
    ry = [p['y'] for p in right]
    ax.plot(rx, ry, 'r--o', linewidth=1.5, markersize=4, label='Right boundary')

    ax.legend()
    plt.tight_layout()
    plt.savefig('test_pcd_processor.png', dpi=150)
    print("  Saved → test_pcd_processor.png")
    plt.show()


def main():
    pcd_path = sys.argv[1] if len(sys.argv) > 1 else '../data/Town01.pcd'

    pcd                      = test_load_and_filter(pcd_path)
    test_kdtree(pcd)
    test_get_z_at(pcd)
    centerline, left, right  = test_boundary_extraction(pcd)
    test_z_enrichment(pcd)
    visualize(pcd, centerline, left, right)

    print("\n=== All tests passed ===\n")


if __name__ == '__main__':
    main()
