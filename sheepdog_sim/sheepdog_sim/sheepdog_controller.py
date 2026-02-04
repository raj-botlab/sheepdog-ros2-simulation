import rclpy
from rclpy.node import Node
import math

from sheepdog_msgs.msg import SheepArray
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# ---- Herding parameters ----
HERD_DIST = 1.0     # distance behind sheep
STOP_DIST = 0.3     # stop distance to avoid oscillation
MAX_SPEED = 2.0     # max sheepdog speed


class SheepdogController(Node):
    def __init__(self):
        super().__init__('sheepdog_controller')

        # -------- Internal state --------
        self.sheep_positions = {}   # name -> (x, y)
        self.sheepdog_pos = None

        # -------- Subscribers --------
        self.create_subscription(
            SheepArray,
            '/sheep_positions',
            self.sheep_callback,
            10
        )

        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.sheepdog_pose_callback,
            10
        )

        # -------- Publisher --------
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # -------- Control loop --------
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Sheepdog autonomous controller running')

    # =====================================================
    # Callbacks
    # =====================================================
    def sheep_callback(self, msg):
        self.sheep_positions.clear()
        for sheep in msg.sheep:
            self.sheep_positions[sheep.name] = (sheep.x, sheep.y)

    def sheepdog_pose_callback(self, msg):
        self.sheepdog_pos = (msg.x, msg.y)

    # =====================================================
    # Main autonomous logic
    # =====================================================
    def control_loop(self):
        # ---- Task finished ----
        if len(self.sheep_positions) == 0:
            self.stop_sheepdog()
            self.get_logger().info('All sheep herded. Task complete.')
            return

        if self.sheepdog_pos is None:
            return

        # ---- STEP 1: Select target sheep (minimum X) ----
        target_name, (xs, ys) = min(
            self.sheep_positions.items(),
            key=lambda item: item[1][0]
        )

        # ---- STEP 2: Compute herding point ----
        herding_x = xs - HERD_DIST
        herding_y = ys

        # ---- STEP 3: Move sheepdog to herding point ----
        xd, yd = self.sheepdog_pos
        dx = herding_x - xd
        dy = herding_y - yd
        dist = math.sqrt(dx * dx + dy * dy)

        cmd = Twist()

        if dist > STOP_DIST:
            speed = min(MAX_SPEED, dist)   # slow down near target
            cmd.linear.x = speed * (dx / dist)
            cmd.linear.y = speed * (dy / dist)
        else:
            # Close enough → stop (prevents oscillation)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        self.cmd_pub.publish(cmd)

    # =====================================================
    # Utility
    # =====================================================
    def stop_sheepdog(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SheepdogController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
