import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from scipy.spatial.transform import Rotation

from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.qos import DurabilityPolicy, HistoryPolicy


class SquareDriver(Node):
    def __init__(self):
        super().__init__('square_driver')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(
            PoseStamped,
            '/model/robot/pose',
            self.callback,
            qos
        )

        self.linear_speed  = 0.3        # m/s
        self.angular_speed = 0.4        # rad/s
        self.side_length   = 2.0        # m
        self.turn_angle    = np.pi / 2  # 90 độ

        self.x   = None
        self.y   = None
        self.yaw = None

        self.start_x   = None
        self.start_y   = None
        self.start_yaw = None

        self.phase      = 'init'   # init | straight | turn | done
        self.side_count = 0

        self.timer = self.create_timer(0.1, self.run)  # 10Hz

    def get_yaw(self, robot_quat):
        q = robot_quat
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')[2]

    def callback(self, msg: PoseStamped):
        self.x   = msg.pose.position.x
        self.y   = msg.pose.position.y
        self.yaw = self.get_yaw(msg.pose.orientation)

    def run(self):
        if self.x is None:
            return

        msg = Twist()

        if self.phase == 'init':
            self.start_x   = self.x
            self.start_y   = self.y
            self.start_yaw = self.yaw
            self.phase     = 'straight'
            self.get_logger().info('[Start]: Edge: 1/4')

        elif self.phase == 'straight':
            dist = np.sqrt((self.x - self.start_x)**2 + (self.y - self.start_y)**2)
            if dist < self.side_length:
                msg.linear.x = self.linear_speed
            else:
                self.start_yaw = self.yaw
                self.phase     = 'turn'
                self.get_logger().info(f'Move Straight done {dist:.3f}m → Rotate 90°')

        elif self.phase == 'turn':
            turned = self.yaw - self.start_yaw
            turned = (turned + np.pi) % (2 * np.pi) - np.pi

            if abs(turned) < self.turn_angle:
                msg.angular.z = self.angular_speed
            else:
                self.side_count += 1
                self.get_logger().info(f'Rotate done {np.degrees(turned):.1f}° → Edge {self.side_count + 1}/4')

                if self.side_count >= 4:
                    self.phase = 'done'
                else:
                    self.start_x   = self.x
                    self.start_y   = self.y
                    self.start_yaw = self.yaw
                    self.phase     = 'straight'

        elif self.phase == 'done':
            self.pub.publish(Twist())
            self.get_logger().info('Complete Square!')
            self.timer.cancel()
            return

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SquareDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()