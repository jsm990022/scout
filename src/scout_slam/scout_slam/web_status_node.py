#!/usr/bin/env python3

import json
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from scout_msgs.msg import ScoutStatus


class WebStatusNode(Node):
    def __init__(self):
        super().__init__('web_status_node')

        self.robot_last_time = 0.0

        self.status_pub = self.create_publisher(
            String,
            '/web_monitor/status',
            10
        )

        self.scout_status_sub = self.create_subscription(
            ScoutStatus,
            '/scout_status',
            self.scout_status_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.publish_status)

        self.get_logger().info('Web Status Node started.')
        self.get_logger().info('Publishing: /web_monitor/status')

    def scout_status_callback(self, msg):
        self.robot_last_time = time.time()

    def is_robot_online(self):
        return (time.time() - self.robot_last_time) < 3.0

    def has_any_node(self, keywords):
        node_names = self.get_node_names()

        for node_name in node_names:
            for keyword in keywords:
                if keyword in node_name:
                    return True

        return False

    def get_navigation_state(self):
        nav_keywords = [
            'controller_server',
            'planner_server',
            'bt_navigator',
            'amcl'
        ]

        if self.has_any_node(nav_keywords):
            return 'READY'

        return 'OFFLINE'

    def get_slam_state(self):
        slam_keywords = [
            'slam_toolbox'
        ]

        if self.has_any_node(slam_keywords):
            return 'READY'

        return 'OFFLINE'

    def publish_status(self):
        status = {
            'ros': 'CONNECTED',
            'websocket': 'CONNECTED',
            'robot': 'ONLINE' if self.is_robot_online() else 'OFFLINE',
            'navigation': self.get_navigation_state(),
            'slam': self.get_slam_state()
        }

        msg = String()
        msg.data = json.dumps(status)

        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WebStatusNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()