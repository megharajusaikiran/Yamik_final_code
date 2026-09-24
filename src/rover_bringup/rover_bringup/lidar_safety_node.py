#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class LidarSafety(Node):
    def __init__(self):
        super().__init__('lidar_safety_node')

        self.declare_parameter('front_stop_m', 0.45)
        self.declare_parameter('rear_stop_m', 0.35)
        self.declare_parameter('side_stop_m', 0.25)
        self.declare_parameter('sector_half_width_deg', 25.0)

        self.latest = None

        qos_scan = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/lidar/safety_status', 10)

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_scan
        )

        self.create_subscription(
            Twist,
            '/cmd_vel_raw',
            self.cmd_cb,
            10
        )

        self.get_logger().info(
            'LiDAR safety node active: /cmd_vel_raw -> /cmd_vel'
        )

    def scan_cb(self, msg):
        self.latest = msg

    def sector(self, center, width):
        if self.latest is None:
            return math.inf

        best = math.inf

        for i, raw in enumerate(self.latest.ranges):
            distance = float(raw)

            if not math.isfinite(distance):
                continue

            if distance < self.latest.range_min:
                continue

            if distance > self.latest.range_max:
                continue

            angle = self.latest.angle_min + (
                i * self.latest.angle_increment
            )

            difference = math.atan2(
                math.sin(angle - center),
                math.cos(angle - center)
            )

            if abs(difference) <= width:
                best = min(best, distance)

        return best

    def cmd_cb(self, incoming):
        output = Twist()
        output.linear.x = incoming.linear.x
        output.angular.z = incoming.angular.z

        sector_width = math.radians(
            float(
                self.get_parameter(
                    'sector_half_width_deg'
                ).value
            )
        )

        front = self.sector(0.0, sector_width)
        rear = self.sector(math.pi, sector_width)
        left = self.sector(math.pi / 2, sector_width)
        right = self.sector(-math.pi / 2, sector_width)

        front_limit = float(
            self.get_parameter('front_stop_m').value
        )

        rear_limit = float(
            self.get_parameter('rear_stop_m').value
        )

        side_limit = float(
            self.get_parameter('side_stop_m').value
        )

        blocked = []

        if output.linear.x > 0.0 and front < front_limit:
            output.linear.x = 0.0
            blocked.append(
                f'FORWARD BLOCKED {front:.2f}m'
            )

        if output.linear.x < 0.0 and rear < rear_limit:
            output.linear.x = 0.0
            blocked.append(
                f'REVERSE BLOCKED {rear:.2f}m'
            )

        if output.angular.z > 0.0 and left < side_limit:
            output.angular.z = 0.0
            blocked.append(
                f'LEFT TURN BLOCKED {left:.2f}m'
            )

        if output.angular.z < 0.0 and right < side_limit:
            output.angular.z = 0.0
            blocked.append(
                f'RIGHT TURN BLOCKED {right:.2f}m'
            )

        self.pub.publish(output)

        status = String()

        if blocked:
            status.data = ' | '.join(blocked)
        else:
            status.data = (
                f'CLEAR '
                f'F:{self.fmt(front)} '
                f'R:{self.fmt(rear)} '
                f'L:{self.fmt(left)} '
                f'Rt:{self.fmt(right)}'
            )

        self.status_pub.publish(status)

    @staticmethod
    def fmt(value):
        if math.isfinite(value):
            return f'{value:.2f}m'

        return '--'

def main(args=None):
    rclpy.init(args=args)
    node = LidarSafety()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
