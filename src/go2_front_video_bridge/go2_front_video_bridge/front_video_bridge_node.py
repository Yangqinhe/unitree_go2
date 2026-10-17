from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from unitree_go.msg import Go2FrontVideoData

try:
    import cv2
    import numpy as np
except ImportError:
    cv2 = None
    np = None


class Go2FrontVideoBridge(Node):
    """Bridge Unitree Go2 front video bytes to standard ROS image topics."""

    def __init__(self):
        super().__init__("go2_front_video_bridge")

        self.declare_parameter("input_topic", "/frontvideostream")
        self.declare_parameter("image_topic", "/go2/front_camera/image_raw")
        self.declare_parameter("compressed_topic", "/go2/front_camera/image_compressed")
        self.declare_parameter("camera_info_topic", "/go2/front_camera/camera_info")
        self.declare_parameter("frame_id", "go2_front_camera")
        self.declare_parameter("stream_field", "video720p")
        self.declare_parameter("compressed_format", "jpeg")
        self.declare_parameter("publish_raw_image", True)
        self.declare_parameter("publish_camera_info", True)
        self.declare_parameter("warn_every_n_failures", 60)
        self.declare_parameter("width_720p", 1280)
        self.declare_parameter("height_720p", 720)
        self.declare_parameter("width_360p", 640)
        self.declare_parameter("height_360p", 360)
        self.declare_parameter("width_180p", 320)
        self.declare_parameter("height_180p", 180)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.compressed_topic = str(self.get_parameter("compressed_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.stream_field = str(self.get_parameter("stream_field").value)
        self.compressed_format = str(self.get_parameter("compressed_format").value)
        self.publish_raw_image = bool(self.get_parameter("publish_raw_image").value)
        self.publish_camera_info = bool(self.get_parameter("publish_camera_info").value)
        self.warn_every_n_failures = int(self.get_parameter("warn_every_n_failures").value)

        self.default_sizes = {
            "video720p": (
                int(self.get_parameter("width_720p").value),
                int(self.get_parameter("height_720p").value),
            ),
            "video360p": (
                int(self.get_parameter("width_360p").value),
                int(self.get_parameter("height_360p").value),
            ),
            "video180p": (
                int(self.get_parameter("width_180p").value),
                int(self.get_parameter("height_180p").value),
            ),
        }

        if self.stream_field not in self.default_sizes:
            raise ValueError(
                "stream_field must be one of video720p, video360p, video180p"
            )

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Go2FrontVideoData, self.input_topic, self._on_video, sensor_qos
        )
        self.compressed_pub = self.create_publisher(
            CompressedImage, self.compressed_topic, 10
        )
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )

        self.frame_count = 0
        self.decode_failures = 0
        self.last_size = self.default_sizes[self.stream_field]

        if self.publish_raw_image and cv2 is None:
            self.get_logger().warn(
                "python3-opencv is not available; only CompressedImage will be published. "
                "Install with: sudo apt install python3-opencv"
            )

        self.get_logger().info(
            f"Go2 front video bridge: {self.input_topic}/{self.stream_field} -> "
            f"{self.image_topic}, {self.compressed_topic}"
        )

    def _on_video(self, msg: Go2FrontVideoData):
        data = self._selected_bytes(msg)
        if not data:
            return

        stamp = self._stamp_from_time_frame(msg.time_frame)
        self.frame_count += 1

        compressed = CompressedImage()
        compressed.header.stamp = stamp
        compressed.header.frame_id = self.frame_id
        compressed.format = self.compressed_format
        compressed.data = data
        self.compressed_pub.publish(compressed)

        image = self._decode_to_image(data)
        if image is not None:
            image.header.stamp = stamp
            image.header.frame_id = self.frame_id
            self.last_size = (image.width, image.height)
            self.image_pub.publish(image)
            if self.publish_camera_info:
                self.camera_info_pub.publish(self._make_camera_info(stamp, image.width, image.height))
        elif self.publish_camera_info:
            width, height = self.last_size
            self.camera_info_pub.publish(self._make_camera_info(stamp, width, height))

    def _selected_bytes(self, msg: Go2FrontVideoData) -> bytes:
        value = getattr(msg, self.stream_field)
        if isinstance(value, bytes):
            return value
        return bytes(value)

    def _stamp_from_time_frame(self, time_frame: int):
        # Unitree's time_frame is not documented in the .msg. Use ROS receive
        # time for a consistent image stream.
        return self.get_clock().now().to_msg()

    def _decode_to_image(self, data: bytes) -> Optional[Image]:
        if not self.publish_raw_image or cv2 is None or np is None:
            return None

        if not self._looks_like_jpeg(data):
            self.decode_failures += 1
            if self._should_warn():
                self.get_logger().warn(
                    "front video bytes do not look like JPEG. Published CompressedImage only. "
                    "If the stream is H264/H265, use a video decoder bridge before RViz2."
                )
            return None

        array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            self.decode_failures += 1
            if self._should_warn():
                self.get_logger().warn("OpenCV failed to decode front video JPEG frame")
            return None

        height, width, channels = frame.shape
        image = Image()
        image.height = height
        image.width = width
        image.encoding = "bgr8"
        image.is_bigendian = 0
        image.step = width * channels
        image.data = frame.tobytes()
        return image

    @staticmethod
    def _looks_like_jpeg(data: bytes) -> bool:
        return len(data) >= 4 and data[0] == 0xFF and data[1] == 0xD8

    def _should_warn(self) -> bool:
        if self.warn_every_n_failures <= 0:
            return False
        return self.decode_failures == 1 or self.decode_failures % self.warn_every_n_failures == 0

    def _make_camera_info(self, stamp, width: int, height: int) -> CameraInfo:
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.frame_id
        info.width = int(width)
        info.height = int(height)
        # Intrinsics are unknown from Go2FrontVideoData. Publish a valid
        # placeholder CameraInfo so consumers can synchronize with the image.
        fx = float(width)
        fy = float(width)
        cx = float(width) / 2.0
        cy = float(height) / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        return info


def main(args=None):
    rclpy.init(args=args)
    node = Go2FrontVideoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
