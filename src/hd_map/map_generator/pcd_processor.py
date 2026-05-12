import open3d as o3d
import numpy as np
from scipy.spatial import cKDTree
import math


class PCDProcessor:
    """Loads and processes a PCD point cloud for HD map generation."""

    def __init__(self):
        self.ground_points = None   # numpy (N, 3) after filtering
        self.kdtree        = None   # 2D KD-tree on (x, y) in CARLA frame

    def load_and_filter(self,
                        pcd_path:    str,
                        z_tolerance: float = 0.3) -> bool:
        """
        Load PCD file and filter to road-surface ground points.

        Bounds are computed automatically from the PCD itself:
        1. Detect ground Z from the histogram peak of all points
        2. Keep only points within z_tolerance of ground level
        3. The remaining points ARE the road surface: no XY filter needed

        PCD Y axis is negated relative to CARLA world frame.
        """
        pcd = o3d.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            print(f"[PCDProcessor] Failed to load: {pcd_path}")
            return False

        pts = np.asarray(pcd.points)
        print(f"[PCDProcessor] Loaded        : {len(pts)} points")

        # Step 1: detect ground Z from histogram of ALL points
        hist, edges = np.histogram(pts[:, 2], bins=1000)
        peak_idx    = np.argmax(hist)
        ground_z    = (edges[peak_idx] + edges[peak_idx + 1]) / 2.0
        print(f"[PCDProcessor] Ground Z       : {ground_z:.4f} m")

        # Step 2: keep only points near ground: this IS the road surface
        mask_z             = np.abs(pts[:, 2] - ground_z) <= z_tolerance
        self.ground_points = pts[mask_z]
        print(f"[PCDProcessor] After Z filter : {len(self.ground_points)} points")

        # Step 3: report the auto-detected bounds from ground points
        gx = self.ground_points[:, 0]
        gy = self.ground_points[:, 1]   # still in PCD frame
        print(f"[PCDProcessor] Ground X range : {gx.min():.2f} to {gx.max():.2f}")
        print(f"[PCDProcessor] Ground Y range : {gy.min():.2f} to {gy.max():.2f}")

        return True

    def build_kdtree(self) -> bool:
        """
        Build a 2D KD-tree on (x, y) in CARLA frame.

        PCD Y is negated vs CARLA, so we flip Y before indexing.
        All subsequent queries use CARLA (x, y) directly.
        """
        if self.ground_points is None:
            print("[PCDProcessor] No ground points available.")
            return False

        xy          = np.column_stack([self.ground_points[:, 0],
                                       self.ground_points[:, 1]])
        self.kdtree = cKDTree(xy)
        print(f"[PCDProcessor] KD-tree built  : {len(xy)} points")
        return True

    def get_z_at(self, x: float, y: float, max_dist: float = 2.0) -> float:
        """
        Return ground elevation at CARLA position (x, y).
        Returns 0.0 if no point found within max_dist metres.
        """
        if self.kdtree is None:
            return 0.0

        dist, idx = self.kdtree.query([[x, y]], k=1)
        if dist[0] > max_dist:
            return 0.0

        return round(float(self.ground_points[idx[0], 2]), 4)

    def extract_lane_boundary(self,
                               centerline:  list,
                               lane_width:  float,
                               side:        str) -> list:
        """
        Compute dense boundary points for one side of a lane.

        Projects each centerline point left or right by half the lane
        width along the perpendicular direction, then snaps Z from PCD.

        Args:
            centerline : list of dicts {x, y, z, heading_rad}
            lane_width : lane width in metres
            side       : 'left' or 'right'

        Returns:
            list of dicts [{x, y, z}, ...]
        """
        if side not in ('left', 'right'):
            raise ValueError("side must be 'left' or 'right'")

        half_width = lane_width / 2.0
        sign       = 1.0 if side == 'left' else -1.0
        boundary   = []

        for pt in centerline:
            # Rotate heading 90° in the chosen direction
            perp = pt['heading_rad'] + sign * (math.pi / 2.0)

            bx = pt['x'] + math.cos(perp) * half_width
            by = pt['y'] + math.sin(perp) * half_width
            bz = self.get_z_at(bx, by)

            boundary.append({"x": round(bx, 4),
                             "y": round(by, 4),
                             "z": round(bz, 4)})

        return boundary

    def enrich_centerline_z(self, centerline: list) -> list:
        """Replace z in each centerline point with real PCD ground height."""
        for pt in centerline:
            pt['z'] = self.get_z_at(pt['x'], pt['y'])
        return centerline