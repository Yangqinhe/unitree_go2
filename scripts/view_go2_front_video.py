#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from unitree_go.msg import Go2FrontVideoData

try:
    import cv2
    import numpy as np
except ImportError as exc:
    print("Missing dependency: python3-opencv and numpy are required.", file=sys.stderr)
    print("Install with: sudo apt install -y python3-opencv python3-numpy", file=sys.stderr)
    raise exc


class Go2FrontVideoViewer(Node):
    def __init__(self, topic: str, stream_field: str, window_name: str):
        super().__init__("go2_front_video_viewer")
        self.topic = topic
        self.stream_field = stream_field
        self.window_name = window_name
        self.frame_count = 0
        self.decode_failures = 0
        self.last_log_time = time.monotonic()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            Go2FrontVideoData, self.topic, self.on_msg, qos
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"Viewing {self.topic}/{self.stream_field}. Press q or Esc in the image window to quit."
        )

    def on_msg(self, msg: Go2FrontVideoData):
        data = bytes(getattr(msg, self.stream_field))
        if not data:
            return

        if not self.looks_like_jpeg(data):
            self.decode_failures += 1
            if self.decode_failures == 1 or self.decode_failures % 60 == 0:
                prefix = " ".join(f"{b:02x}" for b in data[:16])
                self.get_logger().warn(
                    f"{self.stream_field} does not look like JPEG. "
                    f"first_bytes={prefix}. If this is H264/H265, OpenCV imdecode cannot show it directly."
                )
            return

        array = np.frombuffer(data, dtype=np.uint8)
        frame = cv2.imdecode(array, cv2.IMREAD_COLOR)
        if frame is None:
            self.decode_failures += 1
            if self.decode_failures == 1 or self.decode_failures % 60 == 0:
                self.get_logger().warn("OpenCV failed to decode front video frame")
            return

        self.frame_count += 1
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            rclpy.shutdown()
            return

        now = time.monotonic()
        if now - self.last_log_time >= 2.0:
            self.get_logger().info(
                f"frames={self.frame_count}, size={frame.shape[1]}x{frame.shape[0]}"
            )
            self.last_log_time = now

    @staticmethod
    def looks_like_jpeg(data: bytes) -> bool:
        return len(data) >= 4 and data[0] == 0xFF and data[1] == 0xD8


def main():
    parser = argparse.ArgumentParser(description="View Unitree Go2 front video stream.")
    parser.add_argument("--topic", default="/frontvideostream")
    parser.add_argument(
        "--stream",
        default="video720p",
        choices=("video720p", "video360p", "video180p"),
        help="Go2FrontVideoData field to display.",
    )
    parser.add_argument("--window", default="Go2 Front Camera")
    args = parser.parse_args()

    rclpy.init()
    node = Go2FrontVideoViewer(args.topic, args.stream, args.window)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if cv2.getWindowProperty(args.window, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
