import json
import math

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


class TargetTrackerNode(Node):
    """Smooth a detected person point and publish a stable follow target."""

    def __init__(self):
        super().__init__("go2_target_tracker")

        self.declare_parameter("detection_topic", "/go2/detected_person_point")
        self.declare_parameter("target_topic", "/go2/follow_target")
        self.declare_parameter("state_topic", "/go2/follow_target_state")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("alpha", 0.35)
        self.declare_parameter("max_jump_distance", 1.0)
        self.declare_parameter("target_timeout", 0.8)
        self.declare_parameter("search_timeout", 2.0)
        self.declare_parameter("min_x", 0.25)
        self.declare_parameter("max_x", 5.0)
        self.declare_parameter("max_abs_y", 2.5)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("allow_untransformed_detection", True)

        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.target_topic = str(self.get_parameter("target_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.max_jump_distance = float(self.get_parameter("max_jump_distance").value)
        self.target_timeout = float(self.get_parameter("target_timeout").value)
        self.search_timeout = float(self.get_parameter("search_timeout").value)
        self.min_x = float(self.get_parameter("min_x").value)
        self.max_x = float(self.get_parameter("max_x").value)
        self.max_abs_y = float(self.get_parameter("max_abs_y").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.allow_untransformed_detection = bool(
            self.get_parameter("allow_untransformed_detection").value
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pub = self.create_publisher(PointStamped, self.target_topic, 10)
        self.state_pub = self.create_publisher(String, self.state_topic, 10)
        self.detection_sub = self.create_subscription(
            PointStamped, self.detection_topic, self._on_detection, 10
        )
        self.timer = self.create_timer(1.0 / max(publish_rate, 1.0), self._publish)

        self.filtered = None
        self.last_detection_time = None
        self.last_publish_time = None
        self._warned_untransformed = False

        self.get_logger().info(
            f"target tracker: {self.detection_topic} -> {self.target_topic}, frame={self.target_frame}"
        )

    def _on_detection(self, msg: PointStamped):
        point = self._transform_detection(msg)
        if point is None:
            return
        x, y, z = point
        if not self._valid_target(x, y):
            return

        if self.filtered is None:
            self.filtered = [x, y, z]
        else:
            jump = math.sqrt(
                (x - self.filtered[0]) ** 2
                + (y - self.filtered[1]) ** 2
                + (z - self.filtered[2]) ** 2
            )
            alpha = min(self.alpha, 0.15) if jump > self.max_jump_distance else self.alpha
            self.filtered[0] = (1.0 - alpha) * self.filtered[0] + alpha * x
            self.filtered[1] = (1.0 - alpha) * self.filtered[1] + alpha * y
            self.filtered[2] = (1.0 - alpha) * self.filtered[2] + alpha * z

        self.last_detection_time = self.get_clock().now()

    def _transform_detection(self, msg: PointStamped):
        source_frame = msg.header.frame_id.strip("/")
        target_frame = self.target_frame.strip("/")
        point = (msg.point.x, msg.point.y, msg.point.z)
        if not source_frame or source_frame == target_frame:
            return point

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
            return self._transform_point(point, transform)
        except TransformException as exc:
            if not self.allow_untransformed_detection:
                self.get_logger().warn(f"missing TF {source_frame} -> {target_frame}: {exc}")
                return None
            if not self._warned_untransformed:
                self.get_logger().warn(
                    f"missing TF {source_frame} -> {target_frame}; using detection frame as target frame."
                )
                self._warned_untransformed = True
            return point

    @staticmethod
    def _transform_point(point, transform):
        x, y, z = point
        t = transform.transform.translation
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return x + t.x, y + t.y, z + t.z
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        rx = (
            (1 - 2 * qy * qy - 2 * qz * qz) * x
            + (2 * qx * qy - 2 * qz * qw) * y
            + (2 * qx * qz + 2 * qy * qw) * z
        )
        ry = (
            (2 * qx * qy + 2 * qz * qw) * x
            + (1 - 2 * qx * qx - 2 * qz * qz) * y
            + (2 * qy * qz - 2 * qx * qw) * z
        )
        rz = (
            (2 * qx * qz - 2 * qy * qw) * x
            + (2 * qy * qz + 2 * qx * qw) * y
            + (1 - 2 * qx * qx - 2 * qy * qy) * z
        )
        return rx + t.x, ry + t.y, rz + t.z

    def _valid_target(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and abs(y) <= self.max_abs_y

    def _publish(self):
        now = self.get_clock().now()
        visible = False
        age = None
        mode = "lost"
        if self.last_detection_time is not None:
            age = (now - self.last_detection_time).nanoseconds / 1e9
            visible = age <= self.target_timeout
            mode = "tracking" if visible else ("search" if age <= self.search_timeout else "lost")

        if visible and self.filtered is not None:
            msg = PointStamped()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = self.target_frame
            msg.point.x = float(self.filtered[0])
            msg.point.y = float(self.filtered[1])
            msg.point.z = float(self.filtered[2])
            self.target_pub.publish(msg)

        state = {
            "mode": mode,
            "target_visible": visible,
            "target_age_sec": age,
            "target_frame": self.target_frame,
            "target": None
            if self.filtered is None
            else {
                "x": self.filtered[0],
                "y": self.filtered[1],
                "z": self.filtered[2],
            },
        }
        self.state_pub.publish(String(data=json.dumps(state, separators=(",", ":"))))


def main(args=None):
    rclpy.init(args=args)
    node = TargetTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
