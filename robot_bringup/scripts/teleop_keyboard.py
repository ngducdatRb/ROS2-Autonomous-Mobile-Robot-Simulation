#!/usr/bin/env python3
"""
teleop_keyboard.py
Control the differential drive robot using the keyboard via ROS 2.

Use:
    ros2 run <your_package> teleop_keyboard
    or:
    python3 teleop_keyboard.py

Control keys:
    w / ↑  : forward
    s / ↓  : backward
    a / ←  : turn left
    d / →  : turn right
    q      : forward + turn left
    e      : forward + turn right
    z      : backward + turn left
    c      : backward + turn right
    Space  : stop immediately
    t      : increase straight speed
    g      : decrease straight speed
    y      : increase rotation speed
    h      : increase rotation speed
    Ctrl+C : exit
"""

import sys
import tty
import termios
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ─────────────────────────────────────────────
# DEFAULT CONFIG
# ─────────────────────────────────────────────
DEFAULT_LINEAR_SPEED  = 0.3   # m/s
DEFAULT_ANGULAR_SPEED = 0.8   # rad/s
LINEAR_STEP           = 0.05  # linear speed increment/decrement step
ANGULAR_STEP          = 0.1   # rotational speed increment/decrement step
PUBLISH_RATE_HZ       = 10    # Hz — frequency publish /cmd_vel
CMD_VEL_TOPIC         = '/cmd_vel'

# ─────────────────────────────────────────────
# KEYBOARD MAP → (linear_x, angular_z)
# ─────────────────────────────────────────────
KEY_BINDINGS = {
    # WASD
    'w': ( 1.0,  0.0),
    's': (-1.0,  0.0),
    'a': ( 0.0,  1.0),
    'd': ( 0.0, -1.0),
    # Diagonal
    'q': ( 1.0,  1.0),
    'e': ( 1.0, -1.0),
    'z': (-1.0,  1.0),
    'c': (-1.0, -1.0),
    # Stop
    ' ': ( 0.0,  0.0),
}

SPEED_BINDINGS = {
    't': ('linear',  +1),
    'g': ('linear',  -1),
    'y': ('angular', +1),
    'h': ('angular', -1),
}

ARROW_KEYS = {
    '\x1b[A': 'w',  # ↑
    '\x1b[B': 's',  # ↓
    '\x1b[C': 'd',  # →
    '\x1b[D': 'a',  # ←
}

BANNER = """
╔════════════════════════════════════════════════════════
║                  ROBOT TELEOP KEYBOARD
╠════════════════════════════════════════════════════════
║  w/↑ : forward                a/← : turn left
║  s/↓ : backward               d/→ : turn right
║  q   : forward+turn left      z   : backward+turn left
║  e   : forward+turn right     c   : backward+turn right
║  SPACE : STOP
╠════════════════════════════════════════════════════════
║  t/g : increase/decrease straight speed
║  y/h : increase/decrease rotational speed
║  Ctrl+C : EXIT
╚════════════════════════════════════════════════════════
"""


# ─────────────────────────────────────────────
# READ 1 KEY FROM TERMINAL (non-blocking)
# ─────────────────────────────────────────────
def get_key(settings):
    """Read a character from stdin, handle arrow keys as well (escape sequence)."""
    tty.setraw(sys.stdin.fileno())
    try:
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            ch = ch + ch2 + ch3
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return ch


# ─────────────────────────────────────────────
# NODE ROS 2
# ─────────────────────────────────────────────
class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')

        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        self.linear_speed  = DEFAULT_LINEAR_SPEED
        self.angular_speed = DEFAULT_ANGULAR_SPEED

        self._twist = Twist()
        self._lock  = threading.Lock()
        self._running = True

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_cb)

        self.get_logger().info(f"Publish: {CMD_VEL_TOPIC}")
        self.get_logger().info(
            f"Linear speed: {self.linear_speed:.2f} m/s | "
            f"Angular speed: {self.angular_speed:.2f} rad/s"
        )

    # ── Callback timer: publish cmd_vel ──────
    def _publish_cb(self):
        with self._lock:
            self.pub.publish(self._twist)

    # ── Update twist from key ────────────────
    def set_motion(self, linear_x: float, angular_z: float):
        with self._lock:
            self._twist.linear.x  = linear_x  * self.linear_speed
            self._twist.angular.z = angular_z * self.angular_speed

    # ── Stop robot ───────────────────────────
    def stop(self):
        with self._lock:
            self._twist = Twist()
        self.pub.publish(Twist())

    # ── Adjust speed ─────────────────────────
    def adjust_speed(self, axis: str, direction: int):
        if axis == 'linear':
            self.linear_speed = max(0.05, self.linear_speed + direction * LINEAR_STEP)
            print(f"\r  Linear: {self.linear_speed:.2f} m/s             ", flush=True)
        else:
            self.angular_speed = max(0.1, self.angular_speed + direction * ANGULAR_STEP)
            print(f"\r  Angular : {self.angular_speed:.2f} rad/s          ", flush=True)

    def shutdown(self):
        self._running = False
        self.stop()


# ─────────────────────────────────────────────
# KEY READ LOOP (runs on main thread)
# ─────────────────────────────────────────────
def keyboard_loop(node: TeleopKeyboard):
    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    print(f"Linear speed:  {node.linear_speed:.2f} m/s | "
          f"Angular speed: {node.angular_speed:.2f} rad/s\n")

    try:
        while rclpy.ok() and node._running:
            key = get_key(settings)

            if key == '\x03':
                break

            if key in ARROW_KEYS:
                key = ARROW_KEYS[key]

            if key in KEY_BINDINGS:
                lx, az = KEY_BINDINGS[key]
                node.set_motion(lx, az)
                if key == ' ':
                    print('\r  [STOP]                                     ', flush=True)
                else:
                    print(f'\r  [MOVE] linear={lx*node.linear_speed:+.2f} m/s  '
                          f'angular={az*node.angular_speed:+.2f} rad/s    ', flush=True)

            elif key in SPEED_BINDINGS:
                axis, direction = SPEED_BINDINGS[key]
                node.adjust_speed(axis, direction)

            elif key:
                node.stop()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.shutdown()


# ─────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        keyboard_loop(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print("\nExited teleop.")


if __name__ == '__main__':
    main()