import json
import select
import sys
import threading
import time
from dataclasses import dataclass

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from unitree_api.msg import Request

if sys.platform != "win32":
    import termios
    import tty


SPORT_API_BALANCE_STAND = 1002
SPORT_API_STOP_MOVE = 1003
SPORT_API_STAND_UP = 1004
SPORT_API_STAND_DOWN = 1005
SPORT_API_MOVE = 1008


HELP_TEXT = """
Unitree Go2 keyboard control

  w/x : forward/backward
  a/d : left/right
  q/e : turn left/right
  space or s : stop
  +/- : linear speed up/down
  [/] : yaw speed down/up
  b : balance stand
  u : stand up
  c : stand down
  h : help
  Ctrl+C : stop and exit
"""


@dataclass
class Velocity:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    def stopped(self) -> bool:
        return self.x == 0.0 and self.y == 0.0 and self.yaw == 0.0


class KeyReader:
    def __enter__(self):
        if sys.platform == "win32":
            import msvcrt

            self._msvcrt = msvcrt
            return self

        self._fd = sys.stdin.fileno()
        self._old_settings = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        if sys.platform != "win32":
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_settings)

    def read_key(self, timeout_sec=0.1):
        if sys.platform == "win32":
            end_time = time.monotonic() + timeout_sec
            while time.monotonic() < end_time:
                if self._msvcrt.kbhit():
                    char = self._msvcrt.getwch()
                    if char in ("\x00", "\xe0"):
                        self._msvcrt.getwch()
                        return None
                    return char
                time.sleep(0.01)
            return None

        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if readable:
            return sys.stdin.read(1)
        return None


class Go2KeyboardControl(Node):
    def __init__(self):
        super().__init__("go2_keyboard_control")

        self.declare_parameter("topic", "/api/sport/request")
        self.declare_parameter("linear_speed", 0.3)
        self.declare_parameter("yaw_speed", 0.8)
        self.declare_parameter("speed_step", 0.05)
        self.declare_parameter("yaw_step", 0.1)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("command_timeout", 0.5)

        topic = self.get_parameter("topic").value
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.yaw_speed = float(self.get_parameter("yaw_speed").value)
        self.speed_step = float(self.get_parameter("speed_step").value)
        self.yaw_step = float(self.get_parameter("yaw_step").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)

        self.publisher = self.create_publisher(Request, topic, 10)
        self.velocity = Velocity()
        self.last_key_time = 0.0
        self._lock = threading.Lock()

        timer_period = 1.0 / max(publish_rate, 1.0)
        self.timer = self.create_timer(timer_period, self._publish_current_velocity)

        self.get_logger().info(f"Publishing sport requests to {topic}")
        print(HELP_TEXT)

    def handle_key(self, key):
        if key is None:
            return

        key = key.lower()
        with self._lock:
            if key == "w":
                self.velocity = Velocity(x=self.linear_speed)
            elif key == "x":
                self.velocity = Velocity(x=-self.linear_speed)
            elif key == "a":
                self.velocity = Velocity(y=self.linear_speed)
            elif key == "d":
                self.velocity = Velocity(y=-self.linear_speed)
            elif key == "q":
                self.velocity = Velocity(yaw=self.yaw_speed)
            elif key == "e":
                self.velocity = Velocity(yaw=-self.yaw_speed)
            elif key in (" ", "s"):
                self.velocity = Velocity()
                self._publish_stop()
            elif key in ("+", "="):
                self.linear_speed = min(self.linear_speed + self.speed_step, 1.0)
                self.get_logger().info(f"linear_speed={self.linear_speed:.2f}")
            elif key in ("-", "_"):
                self.linear_speed = max(self.linear_speed - self.speed_step, 0.05)
                self.get_logger().info(f"linear_speed={self.linear_speed:.2f}")
            elif key == "[":
                self.yaw_speed = max(self.yaw_speed - self.yaw_step, 0.1)
                self.get_logger().info(f"yaw_speed={self.yaw_speed:.2f}")
            elif key == "]":
                self.yaw_speed = min(self.yaw_speed + self.yaw_step, 2.0)
                self.get_logger().info(f"yaw_speed={self.yaw_speed:.2f}")
            elif key == "b":
                self.velocity = Velocity()
                self._publish_simple(SPORT_API_BALANCE_STAND)
            elif key == "u":
                self.velocity = Velocity()
                self._publish_simple(SPORT_API_STAND_UP)
            elif key == "c":
                self.velocity = Velocity()
                self._publish_simple(SPORT_API_STAND_DOWN)
            elif key == "h":
                print(HELP_TEXT)
            else:
                return

            self.last_key_time = time.monotonic()

    def _publish_current_velocity(self):
        with self._lock:
            velocity = self.velocity
            timed_out = (
                self.command_timeout > 0.0
                and not velocity.stopped()
                and time.monotonic() - self.last_key_time > self.command_timeout
            )
            if timed_out:
                self.velocity = Velocity()
                velocity = self.velocity

        self._publish_move(velocity)

    def _publish_move(self, velocity):
        request = Request()
        request.header.identity.api_id = SPORT_API_MOVE
        request.parameter = json.dumps(
            {"x": velocity.x, "y": velocity.y, "z": velocity.yaw},
            separators=(",", ":"),
        )
        self.publisher.publish(request)

    def _publish_stop(self):
        self._publish_simple(SPORT_API_STOP_MOVE)
        self._publish_move(Velocity())

    def _publish_simple(self, api_id):
        request = Request()
        request.header.identity.api_id = api_id
        self.publisher.publish(request)


def main(args=None):
    rclpy.init(args=args)
    node = Go2KeyboardControl()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        with KeyReader() as key_reader:
            while rclpy.ok():
                key = key_reader.read_key()
                if key == "\x03":
                    break
                node.handle_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()
