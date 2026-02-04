import rclpy
from rclpy.node import Node

import random
import math

from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from sheepdog_msgs.msg import Sheep, SheepArray


# ---- Turtlesim world limits (safe margins) ----
WORLD_MIN = 0.5
WORLD_MAX = 10.5


class SheepController(Node):
    def __init__(self):
        super().__init__('sheep_controller')

        # -------- Services --------
        self.spawn_client = self.create_client(Spawn, '/spawn')
        self.kill_client = self.create_client(Kill, '/kill')

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn service...')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /kill service...')

        # -------- Internal state --------
        self.sheep_names = []
        self.sheep_positions = {}     # name -> (x, y)
        self.sheep_vel_pubs = {}      # name -> publisher

        # -------- Spawn sheep --------
        self.spawn_sheep()

        # -------- Subscribe to sheep poses --------
        for name in self.sheep_names:
            self.create_subscription(
                Pose,
                f'/{name}/pose',
                lambda msg, n=name: self.sheep_pose_callback(msg, n),
                10
            )

            self.sheep_vel_pubs[name] = self.create_publisher(
                Twist,
                f'/{name}/cmd_vel',
                10
            )

        # -------- Sheepdog (turtle1) pose --------
        self.sheepdog_pos = None
        self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.sheepdog_callback,
            10
        )

        # -------- Publish sheep positions --------
        self.sheep_pub = self.create_publisher(
            SheepArray,
            '/sheep_positions',
            10
        )

        # -------- Main loop --------
        self.timer = self.create_timer(0.1, self.update)

        self.get_logger().info('Sheep controller running (fear + despawn + wall-safe)')

    # =====================================================
    # Spawn sheep
    # =====================================================
    def spawn_sheep(self):
        for i in range(3):
            req = Spawn.Request()
            req.x = random.uniform(1.0, 4.0)
            req.y = random.uniform(2.0, 9.0)
            req.theta = 0.0
            req.name = f'sheep{i+1}'

            self.spawn_client.call_async(req)
            self.sheep_names.append(req.name)

            self.get_logger().info(f'Spawned {req.name}')

    # =====================================================
    # Callbacks
    # =====================================================
    def sheep_pose_callback(self, msg, name):
        self.sheep_positions[name] = (msg.x, msg.y)

    def sheepdog_callback(self, msg):
        self.sheepdog_pos = (msg.x, msg.y)

    # =====================================================
    # Main update
    # =====================================================
    def update(self):
        self.check_and_despawn_sheep()
        self.apply_fear_behavior()
        self.publish_sheep_positions()

    # =====================================================
    # Fear behavior (wall-safe)
    # =====================================================
    def apply_fear_behavior(self):
        if self.sheepdog_pos is None:
            return

        xd, yd = self.sheepdog_pos

        for name, (xs, ys) in self.sheep_positions.items():
            dx = xs - xd
            dy = ys - yd
            dist = math.sqrt(dx * dx + dy * dy)

            cmd = Twist()

            if dist < 2.0 and dist > 0.001:
                vx = 1.5 * (dx / dist)
                vy = 1.5 * (dy / dist)

                # ---- Wall / corner protection ----
                if xs > WORLD_MAX and vx > 0:
                    vx = 0.0
                if xs < WORLD_MIN and vx < 0:
                    vx = 0.0
                if ys > WORLD_MAX and vy > 0:
                    vy = 0.0
                if ys < WORLD_MIN and vy < 0:
                    vy = 0.0

                cmd.linear.x = vx
                cmd.linear.y = vy
            else:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0

            self.sheep_vel_pubs[name].publish(cmd)

    # =====================================================
    # Despawn sheep when x > 9.0
    # =====================================================
    def check_and_despawn_sheep(self):
        to_remove = []

        for name, (x, y) in self.sheep_positions.items():
            if x > 9.0:
                self.get_logger().info(f'{name} reached safe zone. Despawning.')
                to_remove.append(name)

        for name in to_remove:
            req = Kill.Request()
            req.name = name
            self.kill_client.call_async(req)

            self.sheep_positions.pop(name, None)
            self.sheep_vel_pubs.pop(name, None)
            if name in self.sheep_names:
                self.sheep_names.remove(name)

    # =====================================================
    # Publish SheepArray
    # =====================================================
    def publish_sheep_positions(self):
        msg = SheepArray()

        for name, (x, y) in self.sheep_positions.items():
            sheep = Sheep()
            sheep.name = name
            sheep.x = x
            sheep.y = y
            msg.sheep.append(sheep)

        self.sheep_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SheepController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
