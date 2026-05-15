#!/usr/bin/env python3
"""
fake_moving_obstacle.py
=======================
Publishes a fake MOVING vehicle ahead of the ego on /fake/objects.
Used to test the FOLLOW_LEADING_VEHICLE behavior.

The fake vehicle:
  - Spawns DISTANCE_M ahead of the ego along its heading
  - Moves at LEAD_SPEED_MPS in the same direction as the ego heading
  - Publishes a RViz marker (green box) so you can see it

Expected behavior:
  - behavior_planner → FOLLOW_LEADING_VEHICLE
  - local_planner    → follows at safe distance behind the fake vehicle
  - controller       → matches leader speed

Press Ctrl+C to remove the vehicle cleanly.

Usage:
  python3 fake_moving_obstacle.py
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Twist, Pose
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class FakeMovingObstacle(Node):

    # How far ahead of the ego to place the vehicle at startup [m]
    DISTANCE_M   = 15.0

    # Speed the fake vehicle moves at [m/s]: keep below normal_speed (8 m/s)
    LEAD_SPEED_MPS = 1.0

    OBSTACLE_ID  = 998

    def __init__(self):
        super().__init__('fake_moving_obstacle')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.objects_pub = self.create_publisher(ObjectArray, '/fake/objects', qos)
        self.marker_pub  = self.create_publisher(Marker, '/fake_obstacle/marker', 10)

        # Vehicle state: updated every publish tick
        self._pos_x: float | None = None
        self._pos_y: float | None = None
        self._pos_z: float = 0.0
        self._yaw:   float = 0.0
        self._last_publish_time: float | None = None

        self._odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self._odom_once_callback,
            qos,
        )
        self.get_logger().info('Waiting for odometry...')

    # ── Initialise position from first odometry message ──────────────────────

    def _odom_once_callback(self, msg: Odometry) -> None:
        ego_x = msg.pose.pose.position.x
        ego_y = msg.pose.pose.position.y
        self._pos_z = msg.pose.pose.position.z + 0.75

        q = msg.pose.pose.orientation
        self._yaw = 2.0 * math.atan2(q.z, q.w)

        # Place vehicle DISTANCE_M ahead along ego heading
        self._pos_x = ego_x + self.DISTANCE_M * math.cos(self._yaw)
        self._pos_y = ego_y + self.DISTANCE_M * math.sin(self._yaw)

        self.destroy_subscription(self._odom_sub)

        self.get_logger().info(
            f'Moving vehicle spawned at x={self._pos_x:.2f}, y={self._pos_y:.2f}, '
            f'yaw={math.degrees(self._yaw):.1f}°, speed={self.LEAD_SPEED_MPS} m/s'
        )
        self.get_logger().info('Press Ctrl+C to remove the vehicle cleanly.')

        self._last_publish_time = self.get_clock().now().nanoseconds * 1e-9
        self.create_timer(0.1, self._publish)

    # ── Main publish loop ─────────────────────────────────────────────────────

    def _publish(self) -> None:
        if self._pos_x is None:
            return

        # Advance position: integrate constant velocity along heading
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        dt = now_sec - self._last_publish_time
        self._last_publish_time = now_sec

        self._pos_x += self.LEAD_SPEED_MPS * math.cos(self._yaw) * dt
        self._pos_y += self.LEAD_SPEED_MPS * math.sin(self._yaw) * dt

        now = self.get_clock().now().to_msg()

        # ── Build pose ────────────────────────────────────────────────────────
        pose = Pose()
        pose.position.x    = self._pos_x
        pose.position.y    = self._pos_y
        pose.position.z    = self._pos_z
        pose.orientation.z = math.sin(self._yaw / 2.0)
        pose.orientation.w = math.cos(self._yaw / 2.0)

        # ── ObjectArray ───────────────────────────────────────────────────────
        obj_msg = ObjectArray()
        obj_msg.header.stamp    = now
        obj_msg.header.frame_id = 'map'

        obj = Object()
        obj.header.stamp      = now
        obj.header.frame_id   = 'map'
        obj.id                = self.OBSTACLE_ID
        obj.detection_level   = Object.OBJECT_TRACKED
        obj.object_classified = True
        obj.classification    = Object.CLASSIFICATION_CAR
        obj.pose              = pose

        # Velocity in map frame along heading
        obj.twist = Twist()
        obj.twist.linear.x = self.LEAD_SPEED_MPS * math.cos(self._yaw)
        obj.twist.linear.y = self.LEAD_SPEED_MPS * math.sin(self._yaw)

        obj.shape.type       = SolidPrimitive.BOX
        obj.shape.dimensions = [4.5, 2.0, 1.5]

        obj_msg.objects.append(obj)
        self.objects_pub.publish(obj_msg)

        # ── RViz marker (green box) ───────────────────────────────────────────
        marker = Marker()
        marker.header.stamp    = now
        marker.header.frame_id = 'map'
        marker.ns              = 'fake_moving_obstacle'
        marker.id              = self.OBSTACLE_ID
        marker.type            = Marker.CUBE
        marker.action          = Marker.ADD
        marker.pose            = pose
        marker.scale.x         = 4.5
        marker.scale.y         = 2.0
        marker.scale.z         = 1.5
        marker.color.r         = 0.0
        marker.color.g         = 1.0   # green = moving vehicle
        marker.color.b         = 0.0
        marker.color.a         = 0.8
        marker.lifetime        = Duration(sec=0, nanosec=500_000_000)
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'[/fake/objects] moving vehicle @ x={self._pos_x:.2f}, y={self._pos_y:.2f} '
            f'speed={self.LEAD_SPEED_MPS} m/s',
            throttle_duration_sec=2.0,
        )

    # ── Clean shutdown ────────────────────────────────────────────────────────

    def clear_vehicle(self) -> None:
        """Publish an empty ObjectArray so subscribers clear the vehicle immediately."""
        now = self.get_clock().now().to_msg()

        empty_msg = ObjectArray()
        empty_msg.header.stamp    = now
        empty_msg.header.frame_id = 'map'

        delete_marker = Marker()
        delete_marker.header.stamp    = now
        delete_marker.header.frame_id = 'map'
        delete_marker.ns              = 'fake_moving_obstacle'
        delete_marker.id              = self.OBSTACLE_ID
        delete_marker.action          = Marker.DELETE

        for _ in range(5):
            self.objects_pub.publish(empty_msg)
            self.marker_pub.publish(delete_marker)
            time.sleep(0.05)

        self.get_logger().info('Moving vehicle cleared from all subscribers.')


def main():
    rclpy.init()
    node = FakeMovingObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.clear_vehicle()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()