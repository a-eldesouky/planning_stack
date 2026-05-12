import json
import math
from map_generator.pcd_processor  import PCDProcessor
from map_generator.carla_extractor import CarlaExtractor


class MapBuilder:
    """
    Assembles the HD map from PCD geometry and CARLA semantic data.
    Produces a Lanelet2-inspired JSON map file.
    """

    def __init__(self, pcd_processor: PCDProcessor,
                 carla_extractor: CarlaExtractor):
        self.pcd       = pcd_processor
        self.carla     = carla_extractor
        self.map_dict : dict = {}

    def build(self) -> dict:
        """
        Run all build stages in order and return the complete map dict.

        Order matters:
          1. Extract CARLA data (junctions before TLs for back-filling)
          2. Enrich geometry with PCD
          3. Assemble JSON sections
        """
        print("[MapBuilder] Extracting centerlines...")
        centerlines = self.carla.get_centerlines()

        print("[MapBuilder] Extracting lane graph...")
        lane_graph  = self.carla.get_lane_graph()

        print("[MapBuilder] Extracting junctions...")
        junctions   = self.carla.get_junctions()

        print("[MapBuilder] Building lanelets...")
        lanelets = self._build_lanelets(centerlines, lane_graph)

        print("[MapBuilder] Building metadata...")
        metadata = self._build_metadata(lanelets, junctions)

        self.map_dict = {
            "metadata":       metadata,
            "lanelets":       lanelets,
            "junctions":      junctions,
        }

        print("[MapBuilder] Build complete.")
        return self.map_dict

    def save(self, output_path: str):
        """Write the map dict to a JSON file with readable formatting."""
        if not self.map_dict:
            print("[MapBuilder] Nothing to save: call build() first.")
            return

        with open(output_path, 'w') as f:
            json.dump(self.map_dict, f, indent=2)

        print(f"[MapBuilder] Saved → {output_path}")

    def _build_lanelets(self, centerlines: dict,
                         lane_graph:  dict) -> list:
        """
        Build one lanelet per (road_id, lane_id) with:
          - centerline points enriched with PCD z-values
          - left and right boundaries from PCD projection
          - graph connectivity from lane_graph
          - traffic light association from lane_to_tl
        """
        predecessors_map = lane_graph['predecessors']
        successors_map   = lane_graph['successors']

        # Build neighbor lookup: (road_id, lane_id±1) → exists?
        all_keys = set(centerlines.keys())

        lanelets = []

        for (road_id, lane_id), centerline in centerlines.items():
            lanelet_id = f"lanelet_{road_id}_{lane_id}"

            # Enrich centerline z from PCD ground points
            enriched_centerline = self.pcd.enrich_centerline_z(
                [pt.copy() for pt in centerline]
            )

            # Lane width from first centerline point
            lane_width = centerline[0]['lane_width'] if centerline else 4.0

            # Extract boundaries from PCD
            left_boundary  = self.pcd.extract_lane_boundary(
                enriched_centerline, lane_width, 'left'
            )
            right_boundary = self.pcd.extract_lane_boundary(
                enriched_centerline, lane_width, 'right'
            )

            # Strip heading_rad and lane_width from stored centerline
            #: planners only need x, y, z, heading_rad
            stored_centerline = [
                {
                    "x":           pt['x'],
                    "y":           pt['y'],
                    "z":           pt['z'],
                    "heading_rad": pt['heading_rad'],
                }
                for pt in enriched_centerline
            ]

            # Junction info from first point
            is_junction = centerline[0]['is_junction']
            junction_id = centerline[0]['junction_id']

            # Lane length: sum of distances between consecutive points
            length_m = self._compute_length(enriched_centerline)

            # Neighbors: same road, adjacent lane id
            left_neighbor  = self._get_neighbor(all_keys, road_id, lane_id, -1)
            right_neighbor = self._get_neighbor(all_keys, road_id, lane_id,  1)

            lanelets.append({
                "id":            lanelet_id,
                "road_id":       road_id,
                "lane_id":       lane_id,
                "is_junction":   is_junction,
                "junction_id":   junction_id,
                "length_m":      round(length_m, 2),
                "centerline":    stored_centerline,
                "left_boundary": left_boundary,
                "right_boundary": right_boundary,
                "predecessors":  predecessors_map.get(lanelet_id, []),
                "successors":    successors_map.get(lanelet_id, []),
                "left_neighbor":  left_neighbor,
                "right_neighbor": right_neighbor,
            })

        print(f"[MapBuilder] Built {len(lanelets)} lanelets")
        return lanelets

    def _build_metadata(self, lanelets: list,
                        junctions:      list) -> dict:
        """Assemble the metadata block with counts and map identity."""

        return {
            "map_name":             self.carla.map_name,
            "version":              "1.0",
            "format":               "Lanelet2-inspired JSON",
            "waypoint_spacing_m":   self.carla.waypoint_spacing,
            "heading_unit":         "radians",
            "total_lanelets":       len(lanelets),
            "total_junctions":      len(junctions),
        }

    def _compute_length(self, centerline: list) -> float:
        """Compute total lane length by summing distances between points."""
        total = 0.0
        for i in range(1, len(centerline)):
            dx     = centerline[i]['x'] - centerline[i-1]['x']
            dy     = centerline[i]['y'] - centerline[i-1]['y']
            total += math.sqrt(dx*dx + dy*dy)
        return total

    def _get_neighbor(self, all_keys: set,
                       road_id:  int,
                       lane_id:  int,
                       offset:   int) -> str:
        """
        Return the lanelet id of the adjacent lane or empty string if none.

        In CARLA: lane_id -1 and +1 are on opposite sides of the road centre.
        Offset -1 goes further from centre, +1 goes toward/across centre.
        """
        neighbor_key = (road_id, lane_id + offset)
        if neighbor_key in all_keys:
            return f"lanelet_{road_id}_{lane_id + offset}"
        return ""