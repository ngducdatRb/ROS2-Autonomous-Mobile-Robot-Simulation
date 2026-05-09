import rclpy
import numpy as np

from dataclasses import dataclass

from rclpy.node import Node
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


# =============================================================================
# HELPERS
# =============================================================================

def quat_to_yaw(q) -> float:
    return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]


def normalize_angle(angle: float) -> float:
    return (angle + np.pi) % (2 * np.pi) - np.pi


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class Pose2D:
    x:   float = 0.0
    y:   float = 0.0
    yaw: float = 0.0

    def snapshot(self, other: 'Pose2D') -> None:
        self.x   = other.x
        self.y   = other.y
        self.yaw = other.yaw

    def distance_to(self, ref: 'Pose2D') -> float:
        return np.hypot(self.x - ref.x, self.y - ref.y)

    def yaw_delta_from(self, ref: 'Pose2D') -> float:
        return abs(normalize_angle(self.yaw - ref.yaw))


@dataclass(frozen=True)
class DriverConfig:
    linear_speed:  float = 0.3
    angular_speed: float = 0.4
    side_length:   float = 2.0
    turn_angle:    float = np.pi / 2
    num_sides:     int   = 4
    control_hz:    float = 10.0


# =============================================================================
# CONSTANTS
# =============================================================================

BEST_EFFORT_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)


# =============================================================================
# NODE
# =============================================================================

class SquareDriver(Node):

    def __init__(self, cfg: DriverConfig = DriverConfig()):
        super().__init__('square_driver')

        self.cfg = cfg

        self.pose:     Pose2D | None = None
        self.ref_pose: Pose2D = Pose2D()

        self.state:      str = 'INIT'
        self.side_index: int = 0

        self.cmd_pub = self.create_publisher(Twist, '/r1/cmd_vel', 10)

        self.create_subscription(
            PoseStamped,
            '/r1/odom/ground_truth',
            self._pose_callback,
            BEST_EFFORT_QOS,
        )

        self.timer = self.create_timer(
            1.0 / self.cfg.control_hz,
            self._control_loop,
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Subscriber callback
    # ─────────────────────────────────────────────────────────────────────────

    def _pose_callback(self, msg: PoseStamped) -> None:
        self.pose = Pose2D(
            x   = msg.pose.position.x,
            y   = msg.pose.position.y,
            yaw = quat_to_yaw(msg.pose.orientation),
        )

    # ─────────────────────────────────────────────────────────────────────────
    # Pose helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _snapshot_ref(self) -> None:
        self.ref_pose.snapshot(self.pose)

    def _traveled_distance(self) -> float:
        return self.pose.distance_to(self.ref_pose)

    def _rotated_angle(self) -> float:
        return self.pose.yaw_delta_from(self.ref_pose)

    # ─────────────────────────────────────────────────────────────────────────
    # Publisher helpers
    # ─────────────────────────────────────────────────────────────────────────

    def _publish(self, linear: float = 0.0, angular: float = 0.0) -> None:
        cmd = Twist()
        cmd.linear.x  = linear
        cmd.angular.z = angular
        self.cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self._publish()

    # ─────────────────────────────────────────────────────────────────────────
    # FSM handlers
    # ─────────────────────────────────────────────────────────────────────────

    def _handle_init(self) -> None:
        self._snapshot_ref()
        self.state = 'MOVE'
        self.get_logger().info(f'[START] Edge {self.side_index + 1}/{self.cfg.num_sides}')

    def _handle_move(self) -> None:
        if self._traveled_distance() < self.cfg.side_length:
            self._publish(linear=self.cfg.linear_speed)
        else:
            self.get_logger().info(f'[MOVE DONE] {self._traveled_distance():.2f} m')
            self._snapshot_ref()
            self.state = 'TURN'

    def _handle_turn(self) -> None:
        if self._rotated_angle() < self.cfg.turn_angle:
            self._publish(angular=self.cfg.angular_speed)
        else:
            self.side_index += 1
            self.get_logger().info(f'[TURN DONE] Side {self.side_index}/{self.cfg.num_sides}')

            if self.side_index >= self.cfg.num_sides:
                self.state = 'DONE'
            else:
                self._snapshot_ref()
                self.state = 'MOVE'

    def _handle_done(self) -> None:
        self._publish_stop()
        self.get_logger().info('Square completed!')
        self.timer.cancel()

    # ─────────────────────────────────────────────────────────────────────────
    # Control loop
    # ─────────────────────────────────────────────────────────────────────────

    _FSM_HANDLERS = {
        'INIT': '_handle_init',
        'MOVE': '_handle_move',
        'TURN': '_handle_turn',
        'DONE': '_handle_done',
    }

    def _control_loop(self) -> None:
        if self.pose is None:
            return

        handler_name = self._FSM_HANDLERS.get(self.state)
        if handler_name:
            getattr(self, handler_name)()


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
