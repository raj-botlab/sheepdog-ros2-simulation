import rclpy
from rclpy.node import Node
import random
import math

from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from sheepdog_msgs.msg import Sheep, SheepArray


class SheepController(Node):

    def __init__(self):
        super().__init__('sheep_controller')

        self.spawn = self.create_client(Spawn, '/spawn')
        self.kill = self.create_client(Kill, '/kill')

        while not self.spawn.wait_for_service(timeout_sec=1.0):
            pass
        while not self.kill.wait_for_service(timeout_sec=1.0):
            pass

        self.names = []
        self.pos = {}
        self.pub = {}
        self.dog = None

        for i in range(3):
            r = Spawn.Request()
            r.x = random.uniform(1.0, 4.0)
            r.y = random.uniform(2.0, 9.0)
            r.theta = 0.0
            r.name = 'sheep' + str(i+1)
            self.spawn.call_async(r)
            self.names.append(r.name)

        for n in self.names:
            self.create_subscription(
                Pose,
                '/' + n + '/pose',
                lambda m, k=n: self.cb_sheep(m, k),
                10
            )
            self.pub[n] = self.create_publisher(
                Twist,
                '/' + n + '/cmd_vel',
                10
            )

        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.cb_dog,
            10
        )

        self.out = self.create_publisher(
            SheepArray,
            '/sheep_positions',
            10
        )

        self.create_timer(0.1, self.loop)

    def cb_sheep(self, msg, name):
        self.pos[name] = (msg.x, msg.y)

    def cb_dog(self, msg):
        self.dog = (msg.x, msg.y)

    def loop(self):
        self.check_kill()
        self.move_sheep()
        self.send_pos()

    def move_sheep(self):
        if self.dog is None:
            return

        dx_d, dy_d = self.dog

        for n in self.pos:
            x, y = self.pos[n]
            dx = x - dx_d
            dy = y - dy_d
            d = math.sqrt(dx*dx + dy*dy)

            t = Twist()

            if d < 2.0 and d > 0.01:
                vx = 1.5 * dx / d
                vy = 1.5 * dy / d

                if x > 10.5 and vx > 0:
                    vx = 0.0
                if x < 0.5 and vx < 0:
                    vx = 0.0
                if y > 10.5 and vy > 0:
                    vy = 0.0
                if y < 0.5 and vy < 0:
                    vy = 0.0

                t.linear.x = vx
                t.linear.y = vy

            self.pub[n].publish(t)

    def check_kill(self):
        rem = []
        for n in self.pos:
            if self.pos[n][0] > 9.0:
                rem.append(n)

        for n in rem:
            r = Kill.Request()
            r.name = n
            self.kill.call_async(r)
            self.pos.pop(n, None)
            self.pub.pop(n, None)
            if n in self.names:
                self.names.remove(n)

    def send_pos(self):
        m = SheepArray()
        for n in self.pos:
            s = Sheep()
            s.name = n
            s.x = self.pos[n][0]
            s.y = self.pos[n][1]
            m.sheep.append(s)
        self.out.publish(m)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(SheepController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
