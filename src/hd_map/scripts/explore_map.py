import carla

def main():
    client = carla.Client('localhost', 2000)
    client.set_timeout(60.0)

    # Load Town01 explicitly
    print("Loading Town01...")
    world = client.load_world('Town01')
    carla_map = world.get_map()

    print(f"Map name: {carla_map.name}")

    # Waypoints
    waypoints = carla_map.generate_waypoints(0.5)
    print(f"\nTotal waypoints: {len(waypoints)}")
    print("\n--- ALL Waypoints ---")
    for wp in waypoints:
        print(f"  road={wp.road_id:>4}  lane={wp.lane_id:>3}  "
              f"x={wp.transform.location.x:>8.2f}  y={wp.transform.location.y:>8.2f}  "
              f"z={wp.transform.location.z:>5.2f}  "
              f"width={wp.lane_width:.2f}  "
              f"junction={wp.is_junction}  "
              f"junction_id={wp.get_junction().id if wp.is_junction else 'N/A'}")

    # Topology
    topology = carla_map.get_topology()
    print(f"\nTotal topology edges: {len(topology)}")
    print("\n--- ALL Topology Edges ---")
    for start_wp, end_wp in topology:
        print(f"  Road {start_wp.road_id:>4} Lane {start_wp.lane_id:>3} "
              f"({start_wp.transform.location.x:>8.2f}, {start_wp.transform.location.y:>8.2f})"
              f"  ->  "
              f"Road {end_wp.road_id:>4} Lane {end_wp.lane_id:>3} "
              f"({end_wp.transform.location.x:>8.2f}, {end_wp.transform.location.y:>8.2f})")

    # Junctions
    # Collect unique junction IDs from waypoints
    junction_ids = set()
    for wp in waypoints:
        if wp.is_junction:
            junction_ids.add(wp.get_junction().id)

    print(f"\nTotal unique junctions: {len(junction_ids)}")
    print("\n--- ALL Junctions ---")
    for jid in sorted(junction_ids):
        # Get all waypoint pairs inside this junction
        junction_obj = carla_map.get_waypoint_xodr(
            waypoints[[wp.get_junction().id for wp in waypoints if wp.is_junction].index(jid) 
                       if jid in [wp.get_junction().id for wp in waypoints if wp.is_junction] 
                       else 0].road_id,
            waypoints[0].lane_id, 0.0
        )
        print(f"  Junction ID: {jid}")

    # Traffic Lights
    traffic_lights = world.get_actors().filter('traffic.traffic_light')
    print(f"\nTotal traffic lights: {len(traffic_lights)}")
    print("\n--- ALL Traffic Lights ---")
    for tl in traffic_lights:
        loc = tl.get_location()
        # Get waypoints affected by this traffic light
        affected = tl.get_affected_lane_waypoints()
        affected_str = ", ".join(
            f"road={wp.road_id} lane={wp.lane_id}" for wp in affected
        )
        print(f"  TL id={tl.id:>6}  "
              f"x={loc.x:>8.2f}  y={loc.y:>8.2f}  z={loc.z:>5.2f}  "
              f"state={tl.get_state()}  "
              f"affects=[{affected_str}]")

if __name__ == '__main__':
    main()