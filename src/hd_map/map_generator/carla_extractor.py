import carla
import math
from collections import defaultdict


class CarlaExtractor:
    """Extracts semantic map data from a running CARLA simulation."""

    def __init__(self, map_name: str = 'Town01',
                 waypoint_spacing: float = 2.0):
        """
        Args:
            map_name         : CARLA map to load (e.g. 'Town01', 'Town05')
            waypoint_spacing : metres between waypoints along each lane
        """
        self.map_name         = map_name
        self.waypoint_spacing = waypoint_spacing

        self.client    = None
        self.world     = None
        self.carla_map = None

        self._topology  = None
        self._waypoints = None

    def connect(self, host: str = 'localhost', port: int = 2000) -> bool:
        """
        Connect to CARLA, load the requested map, and cache
        topology and waypoints so they are not re-fetched later.
        """
        try:
            self.client = carla.Client(host, port)
            self.client.set_timeout(60.0)

            self.world = self.client.get_world()
            if self.map_name not in self.world.get_map().name:
                print(f"[CarlaExtractor] Loading {self.map_name}...")
                self.world = self.client.load_world(self.map_name)

            self.carla_map = self.world.get_map()
            print(f"[CarlaExtractor] Connected to: {self.carla_map.name}")

            print(f"[CarlaExtractor] Caching topology and waypoints "
                  f"(spacing={self.waypoint_spacing}m)...")
            self._topology  = self.carla_map.get_topology()
            self._waypoints = self.carla_map.generate_waypoints(
                self.waypoint_spacing
            )
            print(f"[CarlaExtractor]   Topology edges : {len(self._topology)}")
            print(f"[CarlaExtractor]   Waypoints      : {len(self._waypoints)}")
            return True

        except Exception as e:
            print(f"[CarlaExtractor] Connection failed: {e}")
            return False

    def get_centerlines(self) -> dict:
        """
        Extract ordered centerline points for every lane in the map.

        Walks each lane from its topology start waypoint, collecting
        a point every waypoint_spacing metres along the lane centre.

        Returns:
            dict: (road_id, lane_id) -> list of centerline point dicts
                  each point: {x, y, z, heading_rad, lane_width,
                               is_junction, junction_id}
        """
        centerlines = {}

        for start_wp, _ in self._topology:
            key = (start_wp.road_id, start_wp.lane_id)
            if key in centerlines:
                continue

            points    = []
            current   = start_wp
            visited_s = set()

            while current is not None:
                s = round(current.s, 1)
                if s in visited_s:
                    break
                visited_s.add(s)
                
                ros_x, ros_y, ros_yaw = self._carla_to_ros_xyyaw(
                    current.transform.location.x,
                    current.transform.location.y,
                    math.radians(current.transform.rotation.yaw)
                )

                points.append({
                    "x":           ros_x,
                    "y":           ros_y,
                    "z":           round(current.transform.location.z, 4),
                    "heading_rad": ros_yaw,
                    "lane_width":  round(current.lane_width, 4),
                    "is_junction": current.is_junction,
                    "junction_id": (current.get_junction().id
                                    if current.is_junction else -1),
                })

                # Stay on the same road and lane
                nexts     = current.next(self.waypoint_spacing)
                same_lane = [
                    n for n in nexts
                    if n.road_id == start_wp.road_id
                    and n.lane_id == start_wp.lane_id
                ]
                current = same_lane[0] if same_lane else None

            centerlines[key] = points

        print(f"[CarlaExtractor] Extracted {len(centerlines)} centerlines")
        return centerlines

    def get_lane_graph(self) -> dict:
        """
        Build lane connectivity from topology edges.

        Each topology edge (start_wp → end_wp) represents a directed
        connection between two lanes.

        Returns:
            dict with keys:
                'predecessors': {lane_id_str -> [lane_id_str, ...]}
                'successors':   {lane_id_str -> [lane_id_str, ...]}
        """
        predecessors = defaultdict(set)
        successors   = defaultdict(set)

        for start_wp, end_wp in self._topology:
            src = f"lanelet_{start_wp.road_id}_{start_wp.lane_id}"
            dst = f"lanelet_{end_wp.road_id}_{end_wp.lane_id}"
            successors[src].add(dst)
            predecessors[dst].add(src)

        return {
            "predecessors": {k: sorted(v) for k, v in predecessors.items()},
            "successors":   {k: sorted(v) for k, v in successors.items()},
        }

    def get_junctions(self) -> list:
        """
        Extract metadata for every junction in the map.

        For each junction: computes center position, classifies type
        (T-junction / 4-way), lists connecting external roads and
        internal lane ids.

        Returns:
            list of junction dicts
        """
        junction_wps   = defaultdict(list)
        junction_lanes = defaultdict(set)

        for wp in self._waypoints:
            if wp.is_junction:
                jid = wp.get_junction().id
                junction_wps[jid].append(wp)
                junction_lanes[jid].add(
                    f"lanelet_{wp.road_id}_{wp.lane_id}"
                )

        # Collect external roads connecting to each junction
        junction_roads = defaultdict(set)
        for start_wp, end_wp in self._topology:
            if not start_wp.is_junction and end_wp.is_junction:
                junction_roads[end_wp.get_junction().id].add(start_wp.road_id)
            elif start_wp.is_junction and not end_wp.is_junction:
                junction_roads[start_wp.get_junction().id].add(end_wp.road_id)

        junctions = []

        for jid in sorted(junction_wps.keys()):
            wps     = junction_wps[jid]
            cx      = sum(w.transform.location.x for w in wps) / len(wps)
            cy      = sum(w.transform.location.y for w in wps) / len(wps)
            n_roads = len(junction_roads[jid])
            j_type  = (
                "T-junction" if n_roads == 3 else
                "4-way"      if n_roads == 4 else
                f"other_{n_roads}"
            )

            junctions.append({
                "id":                jid,
                "center_x":         round(cx, 4),
                "center_y":         round(-cy, 4),  # CARLA y is left-handed, flip to right-handed
                "type":             j_type,
                "connecting_roads": sorted(junction_roads[jid]),
                "internal_lane_ids": sorted(junction_lanes[jid]),
            })

        print(f"[CarlaExtractor] Extracted {len(junctions)} junctions")
        return junctions
    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


    def _carla_to_ros_xyyaw(self, x: float, y: float, yaw: float):
        return (
            round(x, 4),
            round(-y, 4),
            round(self._normalize_angle(-yaw), 6),
        )