# RouteWaypoint.msg

float64 x                   # [m] map frame
float64 y                   # [m] map frame
float64 z                   # [m] map frame
float64 heading             # [rad] forward direction of travel, [0, 2π)
float64 curvature           # [1/m] curvature of the path at this waypoint
float64 road_half_width     # [m] half-width of the road at this waypoint
float64 lane_half_width     # [m] half-width of the lane at this waypoint
bool    is_junction         # true if this waypoint is within a junction area, false otherwise
