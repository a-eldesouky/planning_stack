#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from derived_object_msgs.msg import ObjectArray, Object
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Twist, Pose
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class FakeStaticObstacle(Node):

    DISTANCE_M  = 7.0
    OBSTACLE_ID = 999

    def __init__(self):
        super().__init__('fake_static_obstacle')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.objects_pub = self.create_publisher(ObjectArray, '/fake/objects', qos)
        self.marker_pub  = self.create_publisher(Marker, '/fake_obstacle/marker', 10)

        # Fixed pose: set once when first odometry arrives
        self._obstacle_pose: Pose | None = None

        self._odom_sub = self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self._odom_once_callback,
            qos,
        )
        self.get_logger().info('Waiting for odometry...')

    def _odom_once_callback(self, msg: Odometry) -> None:
        ego_x = msg.pose.pose.position.x
        ego_y = msg.pose.pose.position.y
        ego_z = msg.pose.pose.position.z
        q     = msg.pose.pose.orientation

        # Compute yaw from quaternion
        yaw = 2.0 * math.atan2(q.z, q.w)

        # Place obstacle ahead along ego heading
        obs_x = ego_x + self.DISTANCE_M * math.cos(yaw)
        obs_y = ego_y + self.DISTANCE_M * math.sin(yaw)

        pose = Pose()
        pose.position.x = obs_x
        pose.position.y = obs_y
        pose.position.z = ego_z + 0.75

        # CRITICAL: obstacle orientation must match ego heading
        # so the 3-circle model places circles along the travel direction
        pose.orientation.x = 0.0
        pose.orientation.y = 0.0
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

        self._obstacle_pose = pose

        # Destroy subscription: we only needed one message
        self.destroy_subscription(self._odom_sub)

        self.get_logger().info(
            f'Obstacle fixed at x={obs_x:.2f}, y={obs_y:.2f}, yaw={math.degrees(yaw):.1f}°'
        )

        # Start publishing timer now that we have the pose
        self.create_timer(0.1, self._publish)

    def _publish(self) -> None:
        if self._obstacle_pose is None:
            return

        now  = self.get_clock().now().to_msg()
        pose = self._obstacle_pose

        # ObjectArray
        obj_msg = ObjectArray()
        obj_msg.header.stamp    = now
        obj_msg.header.frame_id = 'map'

        obj = Object()
        obj.header.stamp         = now
        obj.header.frame_id      = 'map'
        obj.id                   = self.OBSTACLE_ID
        obj.detection_level      = Object.OBJECT_TRACKED
        obj.object_classified    = True
        obj.classification       = Object.CLASSIFICATION_CAR

        obj.pose  = pose
        obj.twist = Twist()

        # shape: [length_along_heading, width_perpendicular, height]
        # length=4.5 along yaw direction, width=2.0 perpendicular
        obj.shape.type       = SolidPrimitive.BOX
        obj.shape.dimensions = [4.5, 2.0, 1.5]

        obj_msg.objects.append(obj)
        self.objects_pub.publish(obj_msg)

        # RViz marker
        marker = Marker()
        marker.header.stamp    = now
        marker.header.frame_id = 'map'
        marker.ns              = 'fake_obstacle'
        marker.id              = self.OBSTACLE_ID
        marker.type            = Marker.CUBE
        marker.action          = Marker.ADD
        marker.pose            = pose
        marker.scale.x         = 4.5
        marker.scale.y         = 2.0
        marker.scale.z         = 1.5
        marker.color.r         = 1.0
        marker.color.g         = 0.0
        marker.color.b         = 0.0
        marker.color.a         = 0.8
        marker.lifetime        = Duration(sec=0, nanosec=500_000_000)

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'[/fake/objects] obstacle @ x={pose.position.x:.2f}, '
            f'y={pose.position.y:.2f}, yaw={math.degrees(2*math.atan2(pose.orientation.z, pose.orientation.w)):.1f}°',
            throttle_duration_sec=2.0,
        )


def main():
    rclpy.init()
    node = FakeStaticObstacle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()