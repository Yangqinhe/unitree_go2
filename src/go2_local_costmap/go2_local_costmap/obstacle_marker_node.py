import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class ObstacleMarkerNode(Node):
    """Convert obstacle summary JSON into RViz markers."""

    def __init__(self):
        super().__init__("go2_obstacle_marker")

        self.declare_parameter("summary_topic", "/go2/obstacle_summary")
        self.declare_parameter("marker_topic", "/go2/obstacle_markers")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("hard_stop_distance", 0.45)
        self.declare_parameter("slow_down_distance", 1.0)

        self.summary_topic = str(self.get_parameter("summary_topic").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.slow_down_distance = float(self.get_parameter("slow_down_distance").value)

        self.sub = self.create_subscription(String, self.summary_topic, self._on_summary, 10)
        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)

        self.get_logger().info(f"obstacle markers: {self.summary_topic} -> {self.marker_topic}")

    def _on_summary(self, msg: String):
        try:
            summary = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn("invalid obstacle summary JSON")
            return

        now = self.get_clock().now().to_msg()
        markers = MarkerArray()
        markers.markers.extend(self._delete_old_markers(now))

        front = summary.get("nearest_front_m")
        left = summary.get("nearest_left_m")
        right = summary.get("nearest_right_m")
        blocked = bool(summary.get("front_blocked", False))
        slow = bool(summary.get("front_slow", False))
        vx_scale = float(summary.get("suggested_vx_scale", 1.0))
        steer = float(summary.get("suggested_steer_y", 0.0))

        if blocked:
            status = "STOP"
            color = (1.0, 0.0, 0.0, 0.85)
        elif slow:
            status = "SLOW"
            color = (1.0, 0.75, 0.0, 0.85)
        else:
            status = "CLEAR"
            color = (0.0, 0.8, 0.25, 0.85)

        markers.markers.append(
            self._make_cylinder(
                marker_id=1,
                stamp=now,
                x=self.hard_stop_distance,
                y=0.0,
                radius=0.08,
                color=(1.0, 0.0, 0.0, 0.55),
            )
        )
        markers.markers.append(
            self._make_cylinder(
                marker_id=2,
                stamp=now,
                x=self.slow_down_distance,
                y=0.0,
                radius=0.08,
                color=(1.0, 0.75, 0.0, 0.45),
            )
        )

        if front is not None and math.isfinite(float(front)):
            markers.markers.append(
                self._make_sphere(10, now, float(front), 0.0, color)
            )
        if left is not None and math.isfinite(float(left)):
            markers.markers.append(
                self._make_sphere(11, now, max(0.2, float(left)), 0.45, (0.2, 0.6, 1.0, 0.85))
            )
        if right is not None and math.isfinite(float(right)):
            markers.markers.append(
                self._make_sphere(12, now, max(0.2, float(right)), -0.45, (0.2, 0.6, 1.0, 0.85))
            )

        if abs(steer) > 0.01:
            markers.markers.append(self._make_arrow(20, now, steer))

        text = (
            f"{status}  front={self._fmt(front)}m  "
            f"left={self._fmt(left)}m  right={self._fmt(right)}m  vx={vx_scale:.2f}"
        )
        markers.markers.append(self._make_text(30, now, text))
        self.pub.publish(markers)

    def _delete_old_markers(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.action = Marker.DELETEALL
        return [marker]

    def _make_base(self, marker_id, stamp, marker_type):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "go2_obstacles"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def _make_sphere(self, marker_id, stamp, x, y, color):
        marker = self._make_base(marker_id, stamp, Marker.SPHERE)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.12
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.18
        self._set_color(marker, color)
        return marker

    def _make_cylinder(self, marker_id, stamp, x, y, radius, color):
        marker = self._make_base(marker_id, stamp, Marker.CYLINDER)
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.01
        marker.scale.x = radius
        marker.scale.y = 1.4
        marker.scale.z = 0.02
        self._set_color(marker, color)
        return marker

    def _make_arrow(self, marker_id, stamp, steer):
        marker = self._make_base(marker_id, stamp, Marker.ARROW)
        marker.pose.position.x = 0.35
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.25
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.55
        marker.scale.y = 0.06
        marker.scale.z = 0.10
        marker.points = []
        from geometry_msgs.msg import Point

        start = Point()
        start.x = 0.35
        start.y = 0.0
        start.z = 0.25
        end = Point()
        end.x = 0.85
        end.y = 0.35 if steer > 0.0 else -0.35
        end.z = 0.25
        marker.points = [start, end]
        self._set_color(marker, (0.0, 0.9, 1.0, 0.9))
        return marker

    def _make_text(self, marker_id, stamp, text):
        marker = self._make_base(marker_id, stamp, Marker.TEXT_VIEW_FACING)
        marker.pose.position.x = 0.7
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.65
        marker.scale.z = 0.16
        marker.text = text
        self._set_color(marker, (1.0, 1.0, 1.0, 0.95))
        return marker

    @staticmethod
    def _set_color(marker, color):
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = float(color[3])

    @staticmethod
    def _fmt(value):
        if value is None:
            return "--"
        try:
            return f"{float(value):.2f}"
        except (TypeError, ValueError):
            return "--"


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
