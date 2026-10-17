import json
import math
from typing import Iterable, Optional, Tuple

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


Point3 = Tuple[float, float, float]


class LocalCostmapNode(Node):
    """Project a 3D point cloud into a robot-centric 2D local occupancy grid."""

    def __init__(self):
        super().__init__("go2_local_costmap")

        self.declare_parameter("cloud_topic", "/utlidar/cloud_base")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("costmap_topic", "/go2/local_costmap")
        self.declare_parameter("summary_topic", "/go2/obstacle_summary")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("forward_range", 4.5)
        self.declare_parameter("backward_range", 1.5)
        self.declare_parameter("left_range", 3.0)
        self.declare_parameter("right_range", 3.0)
        self.declare_parameter("min_z", -0.15)
        self.declare_parameter("max_z", 1.20)
        self.declare_parameter("min_range", 0.15)
        self.declare_parameter("max_points", 80000)
        self.declare_parameter("inflation_radius", 0.35)
        self.declare_parameter("occupied_value", 100)
        self.declare_parameter("inflated_value", 60)
        self.declare_parameter("front_half_width", 0.35)
        self.declare_parameter("side_inner_y", 0.20)
        self.declare_parameter("side_outer_y", 0.85)
        self.declare_parameter("sector_min_x", 0.20)
        self.declare_parameter("sector_max_x", 1.40)
        self.declare_parameter("hard_stop_distance", 0.45)
        self.declare_parameter("slow_down_distance", 1.00)
        self.declare_parameter("publish_empty_on_timeout", True)
        self.declare_parameter("cloud_timeout", 0.5)
        self.declare_parameter("allow_untransformed_cloud", True)

        self.cloud_topic = str(self.get_parameter("cloud_topic").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.costmap_topic = str(self.get_parameter("costmap_topic").value)
        self.summary_topic = str(self.get_parameter("summary_topic").value)

        self.resolution = float(self.get_parameter("resolution").value)
        self.forward_range = float(self.get_parameter("forward_range").value)
        self.backward_range = float(self.get_parameter("backward_range").value)
        self.left_range = float(self.get_parameter("left_range").value)
        self.right_range = float(self.get_parameter("right_range").value)
        self.width_m = self.left_range + self.right_range
        self.height_m = self.forward_range + self.backward_range
        self.width_cells = max(1, int(math.ceil(self.width_m / self.resolution)))
        self.height_cells = max(1, int(math.ceil(self.height_m / self.resolution)))
        self.origin_x = -self.backward_range
        self.origin_y = -self.right_range

        self.min_z = float(self.get_parameter("min_z").value)
        self.max_z = float(self.get_parameter("max_z").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_points = int(self.get_parameter("max_points").value)
        self.inflation_radius = float(self.get_parameter("inflation_radius").value)
        self.occupied_value = int(self.get_parameter("occupied_value").value)
        self.inflated_value = int(self.get_parameter("inflated_value").value)
        self.front_half_width = float(self.get_parameter("front_half_width").value)
        self.side_inner_y = float(self.get_parameter("side_inner_y").value)
        self.side_outer_y = float(self.get_parameter("side_outer_y").value)
        self.sector_min_x = float(self.get_parameter("sector_min_x").value)
        self.sector_max_x = float(self.get_parameter("sector_max_x").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.slow_down_distance = float(self.get_parameter("slow_down_distance").value)
        self.publish_empty_on_timeout = bool(self.get_parameter("publish_empty_on_timeout").value)
        self.cloud_timeout = float(self.get_parameter("cloud_timeout").value)
        self.allow_untransformed_cloud = bool(self.get_parameter("allow_untransformed_cloud").value)

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, sensor_qos
        )
        self.costmap_pub = self.create_publisher(OccupancyGrid, self.costmap_topic, 5)
        self.summary_pub = self.create_publisher(String, self.summary_topic, 10)
        self.timeout_timer = self.create_timer(0.2, self._check_timeout)
        self.last_cloud_time = None
        self._warned_untransformed = False

        self.inflation_offsets = self._make_inflation_offsets()

        self.get_logger().info(
            "point cloud local costmap: "
            f"{self.cloud_topic} -> {self.costmap_topic}, frame={self.target_frame}, "
            f"size={self.height_m:.1f}m x {self.width_m:.1f}m, res={self.resolution:.2f}m"
        )

    def _make_inflation_offsets(self):
        radius_cells = int(math.ceil(self.inflation_radius / self.resolution))
        offsets = []
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(dx * self.resolution, dy * self.resolution)
                if distance <= self.inflation_radius:
                    offsets.append((dx, dy))
        return offsets

    def _on_cloud(self, cloud_msg: PointCloud2):
        self.last_cloud_time = self.get_clock().now()
        transform = self._lookup_transform(cloud_msg.header.frame_id)
        points = self._iter_points(cloud_msg, transform)

        data = [0] * (self.width_cells * self.height_cells)
        occupied_cells = set()
        nearest_front = math.inf
        nearest_left = math.inf
        nearest_right = math.inf
        used_points = 0
        occupied_points = 0

        for x, y, z in points:
            used_points += 1
            if used_points > self.max_points:
                break
            if not self._point_is_valid(x, y, z):
                continue

            gx, gy = self._world_to_grid(x, y)
            if gx is None:
                continue

            occupied_points += 1
            occupied_cells.add((gx, gy))
            self._update_sector_distances(x, y, nearest := [nearest_front, nearest_left, nearest_right])
            nearest_front, nearest_left, nearest_right = nearest

        for gx, gy in occupied_cells:
            self._set_cell(data, gx, gy, self.occupied_value)

        for gx, gy in occupied_cells:
            for dx, dy in self.inflation_offsets:
                self._set_cell(data, gx + dx, gy + dy, self.inflated_value)
            self._set_cell(data, gx, gy, self.occupied_value)

        self.costmap_pub.publish(self._build_grid(cloud_msg, data))
        self.summary_pub.publish(
            String(
                data=json.dumps(
                    self._build_summary(
                        cloud_msg,
                        used_points,
                        occupied_points,
                        nearest_front,
                        nearest_left,
                        nearest_right,
                    ),
                    separators=(",", ":"),
                )
            )
        )

    def _lookup_transform(self, source_frame: str):
        source_frame = source_frame.strip("/")
        target_frame = self.target_frame.strip("/")
        if not source_frame or source_frame == target_frame:
            return None

        try:
            return self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as exc:
            if not self.allow_untransformed_cloud:
                self.get_logger().warn(f"missing TF {source_frame} -> {target_frame}: {exc}")
                return "skip"
            if not self._warned_untransformed:
                self.get_logger().warn(
                    f"missing TF {source_frame} -> {target_frame}; projecting cloud in its own frame. "
                    "Set allow_untransformed_cloud:=false after TF is configured."
                )
                self._warned_untransformed = True
            return None

    def _iter_points(self, cloud_msg: PointCloud2, transform) -> Iterable[Point3]:
        if transform == "skip":
            return []

        raw_points = point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True
        )
        if transform is None:
            return (self._coerce_point(p) for p in raw_points)
        return (self._transform_point(self._coerce_point(p), transform) for p in raw_points)

    @staticmethod
    def _coerce_point(point) -> Point3:
        try:
            return float(point[0]), float(point[1]), float(point[2])
        except (KeyError, TypeError, ValueError):
            return float(point["x"]), float(point["y"]), float(point["z"])

    @staticmethod
    def _transform_point(point: Point3, transform) -> Point3:
        x, y, z = point
        t = transform.transform.translation
        q = transform.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return x + t.x, y + t.y, z + t.z
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        # Quaternion rotation matrix, then translation.
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

    def _point_is_valid(self, x: float, y: float, z: float) -> bool:
        if z < self.min_z or z > self.max_z:
            return False
        if x < -self.backward_range or x > self.forward_range:
            return False
        if y < -self.right_range or y > self.left_range:
            return False
        if math.hypot(x, y) < self.min_range:
            return False
        return True

    def _world_to_grid(self, x: float, y: float) -> Tuple[Optional[int], Optional[int]]:
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        if gx < 0 or gx >= self.height_cells or gy < 0 or gy >= self.width_cells:
            return None, None
        return gx, gy

    def _set_cell(self, data, gx: int, gy: int, value: int):
        if gx < 0 or gx >= self.height_cells or gy < 0 or gy >= self.width_cells:
            return
        idx = gy * self.height_cells + gx
        data[idx] = max(data[idx], value)

    def _build_grid(self, cloud_msg: PointCloud2, data) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.stamp = cloud_msg.header.stamp
        grid.header.frame_id = self.target_frame
        grid.info.resolution = self.resolution
        grid.info.width = self.height_cells
        grid.info.height = self.width_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid

    def _update_sector_distances(self, x: float, y: float, nearest):
        if x < self.sector_min_x or x > self.sector_max_x:
            return
        distance = math.hypot(x, y)
        if abs(y) <= self.front_half_width:
            nearest[0] = min(nearest[0], distance)
        elif self.side_inner_y <= y <= self.side_outer_y:
            nearest[1] = min(nearest[1], distance)
        elif -self.side_outer_y <= y <= -self.side_inner_y:
            nearest[2] = min(nearest[2], distance)

    def _build_summary(
        self,
        cloud_msg: PointCloud2,
        used_points: int,
        occupied_points: int,
        nearest_front: float,
        nearest_left: float,
        nearest_right: float,
    ):
        front = self._finite_or_none(nearest_front)
        left = self._finite_or_none(nearest_left)
        right = self._finite_or_none(nearest_right)
        blocked = front is not None and front <= self.hard_stop_distance
        slow = front is not None and front <= self.slow_down_distance
        vx_scale = 1.0
        if blocked:
            vx_scale = 0.0
        elif slow and front is not None:
            span = max(self.slow_down_distance - self.hard_stop_distance, 0.01)
            vx_scale = max(0.0, min(1.0, (front - self.hard_stop_distance) / span))

        steer_hint = 0.0
        if blocked or slow:
            left_clear = left if left is not None else self.sector_max_x
            right_clear = right if right is not None else self.sector_max_x
            steer_hint = -1.0 if left_clear < right_clear else 1.0
            if left is not None and right is not None and left <= self.hard_stop_distance and right <= self.hard_stop_distance:
                steer_hint = 0.0

        return {
            "stamp_sec": cloud_msg.header.stamp.sec,
            "frame_id": self.target_frame,
            "source_frame_id": cloud_msg.header.frame_id,
            "used_points": used_points,
            "occupied_points": occupied_points,
            "nearest_front_m": front,
            "nearest_left_m": left,
            "nearest_right_m": right,
            "front_blocked": blocked,
            "front_slow": slow,
            "suggested_vx_scale": vx_scale,
            "suggested_steer_y": steer_hint,
        }

    @staticmethod
    def _finite_or_none(value: float):
        return value if math.isfinite(value) else None

    def _check_timeout(self):
        if not self.publish_empty_on_timeout or self.last_cloud_time is None:
            return
        age = (self.get_clock().now() - self.last_cloud_time).nanoseconds / 1e9
        if age <= self.cloud_timeout:
            return
        now = self.get_clock().now().to_msg()
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = self.target_frame
        grid.info.resolution = self.resolution
        grid.info.width = self.height_cells
        grid.info.height = self.width_cells
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * (self.width_cells * self.height_cells)
        self.costmap_pub.publish(grid)
        self.summary_pub.publish(
            String(
                data=json.dumps(
                    {
                        "stamp_sec": now.sec,
                        "frame_id": self.target_frame,
                        "cloud_timeout": True,
                        "front_blocked": True,
                        "suggested_vx_scale": 0.0,
                        "suggested_steer_y": 0.0,
                    },
                    separators=(",", ":"),
                )
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = LocalCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
