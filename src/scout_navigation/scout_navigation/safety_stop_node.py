import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy


class SafetyStopNode(Node):

    def __init__(self):
        super().__init__('safety_stop_node')

        self.safe_distance = 0.80
        self.half_width = 15
        self.forward_threshold = 0.01

        self.is_front_obstacle_detected = False

        self.latest_cmd = Twist()

        scan_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            scan_qos
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_selected',
            self.cmd_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            'Safety Stop Active | '
            f'stop_distance={self.safe_distance}m | '
            'input=/cmd_vel_selected | '
            'output=/cmd_vel'
        )

    def scan_callback(self, msg):
        center_index = len(msg.ranges) // 2

        start = max(0, center_index - self.half_width)
        end = min(len(msg.ranges), center_index + self.half_width)

        front_ranges = msg.ranges[start:end]

        valid_ranges = [
            r for r in front_ranges
            if r > 0.0 and
            not math.isnan(r) and
            not math.isinf(r)
        ]

        if len(valid_ranges) == 0:
            self.is_front_obstacle_detected = False
            return

        front_min_distance = min(valid_ranges)

        self.is_front_obstacle_detected = (
            front_min_distance < self.safe_distance
        )

    def cmd_callback(self, msg):
        self.latest_cmd = msg

    def timer_callback(self):
        is_moving_forward = (
            self.latest_cmd.linear.x >
            self.forward_threshold
        )

        if self.is_front_obstacle_detected and is_moving_forward:
            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)
            return

        self.cmd_pub.publish(self.latest_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = SafetyStopNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
