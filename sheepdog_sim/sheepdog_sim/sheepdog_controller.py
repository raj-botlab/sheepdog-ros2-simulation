import rclpy
from rclpy.node import Node
import math

from sheepdog_msgs.msg import SheepArray
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class SheepdogController(Node):

    def __init__(self):
        super().__init__('sheepdog_controller')

        self.sheep = {}
        self.x = None
        self.y = None

        self.create_subscription(
            SheepArray,
            '/sheep_positions',
            self.cb_sheep,
            10
        )

        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_pose,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.create_timer(0.1, self.loop)

    def cb_sheep(self, msg):
        self.sheep = {}
        for s in msg.sheep:
            self.sheep[s.name] = (s.x, s.y)

    def cb_pose(self, msg):
        self.x = msg.x
        self.y = msg.y

    def loop(self):
        if len(self.sheep) == 0:
            self.pub.publish(Twist())
            return

        if self.x is None:
            return

        t = None
        tx = 0
        ty = 0

        for k in self.sheep:
            if t is None or self.sheep[k][0] < tx:
                t = k
                tx = self.sheep[k][0]
                ty = self.sheep[k][1]

        hx = tx - 1.0
        hy = ty

        dx = hx - self.x
        dy = hy - self.y
        d = math.sqrt(dx*dx + dy*dy)

        cmd = Twist()

        if d > 0.3:
            v = min(2.0, d)
            cmd.linear.x = v * dx / d
            cmd.linear.y = v * dy / d

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SheepdogController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()

