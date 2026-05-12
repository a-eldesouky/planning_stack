#!/usr/bin/env python3
"""
HD Map Generator for Town01
Combines PCD ground geometry with CARLA semantic data to produce
a Lanelet2-inspired JSON HD map for autonomous driving planning.

Usage:
    python3 generate_hd_map.py [town_name]
    
Example:
    python3 generate_hd_map.py Town01
"""

import sys
import os

from map_generator.pcd_processor   import PCDProcessor
from map_generator.carla_extractor import CarlaExtractor
from map_generator.map_builder     import MapBuilder

def main():
    
    # Step 0: Set paths
    map_name = sys.argv[1] if len(sys.argv) > 1 else "Town01"
    map_resolution = 1.0  # Meters between waypoints
    pcd_path = f"data/{map_name}.pcd"
    output_path = f"maps/{map_name}_hd_map.json"
    
    if not os.path.exists(pcd_path):
        print(f"[Error] PCD file not found: {pcd_path}")
        sys.exit(1)

    # Step 1: Load and process PCD
    print("[1/4] Processing PCD point cloud...")
    pcd = PCDProcessor()

    if not pcd.load_and_filter(pcd_path):
        print("[Error] Failed to load PCD file.")
        sys.exit(1)

    if not pcd.build_kdtree():
        print("[Error] Failed to build KD-tree.")
        sys.exit(1)

    # Step 2: Connect to CARLA
    print("\n[2/4] Connecting to CARLA...")
    carla = CarlaExtractor(map_name=map_name, waypoint_spacing=map_resolution)

    if not carla.connect():
        print("[Error] Failed to connect to CARLA.")
        print("        Make sure CarlaUE4.sh is running on localhost:2000")
        sys.exit(1)

    # Step 3: Build the map
    print("\n[3/4] Building HD map...")
    builder  = MapBuilder(pcd, carla)
    map_dict = builder.build()

    # Step 4: Save
    print("\n[4/4] Saving map...")
    builder.save(output_path)

    # Summary
    meta = map_dict['metadata']
    print("\n" + "=" * 55)
    print("  HD Map Generation Complete")
    print("=" * 55)
    print(f"  Output         : {output_path}")
    print(f"  Lanelets       : {meta['total_lanelets']}")
    print(f"  Junctions      : {meta['total_junctions']}")
    print("=" * 55 + "\n")


if __name__ == '__main__':
    main()