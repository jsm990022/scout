#!/usr/bin/env python3

import os
import re
import json
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Empty, Trigger
from slam_toolbox.srv import SerializePoseGraph


class SlamManagerNode(Node):
    def __init__(self):
        super().__init__('slam_manager_node')

        self.declare_parameter('map_save_dir', os.path.expanduser('~/scout_ws/maps'))
        self.declare_parameter('posegraph_save_dir', os.path.expanduser('~/scout_ws/maps'))

        self.map_save_dir = self.get_parameter('map_save_dir').value
        self.posegraph_save_dir = self.get_parameter('posegraph_save_dir').value

        os.makedirs(self.map_save_dir, exist_ok=True)
        os.makedirs(self.posegraph_save_dir, exist_ok=True)

        self.reset_client = self.create_client(Empty, '/slam_toolbox/reset')
        self.serialize_client = self.create_client(SerializePoseGraph, '/slam_toolbox/serialize_map')

        self.event_pub = self.create_publisher(
            String,
            '/web_slam/event',
            10
        )

        self.save_map_name_sub = self.create_subscription(
            String,
            '/web_slam/save_map_name',
            self.handle_save_map_name,
            10
        )

        self.serialize_posegraph_name_sub = self.create_subscription(
            String,
            '/web_slam/serialize_posegraph_name',
            self.handle_serialize_posegraph_name,
            10
        )

        self.reset_service = self.create_service(
            Trigger,
            '/web_slam/reset',
            self.handle_reset_slam
        )

        self.save_map_service = self.create_service(
            Trigger,
            '/web_slam/save_map',
            self.handle_save_map_auto
        )

        self.serialize_service = self.create_service(
            Trigger,
            '/web_slam/serialize_posegraph',
            self.handle_serialize_posegraph_auto
        )

        self.get_logger().info('SLAM Manager Node started.')
        self.get_logger().info(f'Map save dir: {self.map_save_dir}')
        self.get_logger().info(f'PoseGraph save dir: {self.posegraph_save_dir}')
        self.get_logger().info('Subscribe: /web_slam/save_map_name')
        self.get_logger().info('Subscribe: /web_slam/serialize_posegraph_name')
        self.get_logger().info('Publish: /web_slam/event')

    def publish_event(self, event_type, success, message):
        msg = String()
        msg.data = json.dumps({
            'type': event_type,
            'success': success,
            'message': message
        }, ensure_ascii=False)

        self.event_pub.publish(msg)

    def get_timestamp_name(self, prefix):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        return f'{prefix}_{timestamp}'

    def sanitize_file_name(self, name):
        name = name.strip()
        name = os.path.basename(name)

        if name.endswith('.yaml'):
            name = name[:-5]

        if name.endswith('.pgm'):
            name = name[:-4]

        if name.endswith('.posegraph'):
            name = name[:-10]

        name = re.sub(r'[^a-zA-Z0-9가-힣_\-]', '_', name)
        return name

    def save_map_with_name(self, map_name):
        safe_name = self.sanitize_file_name(map_name)

        if not safe_name:
            return False, 'Failed: map name is empty.'

        map_path = os.path.join(self.map_save_dir, safe_name)

        command = [
            'ros2',
            'run',
            'nav2_map_server',
            'map_saver_cli',
            '-f',
            map_path
        ]

        self.get_logger().info(f'Saving map with command: {" ".join(command)}')

        try:
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=20
            )

            if result.returncode != 0:
                return False, f'Failed to save map: {result.stderr.strip()}'

            return True, f'Map saved: {map_path}.yaml / {map_path}.pgm'

        except subprocess.TimeoutExpired:
            return False, 'Failed: map_saver_cli timeout.'

        except Exception as e:
            return False, f'Failed to save map: {str(e)}'

    def serialize_posegraph_with_name(self, posegraph_name):
        safe_name = self.sanitize_file_name(posegraph_name)

        if not safe_name:
            return False, 'Failed: posegraph name is empty.'

        if not self.serialize_client.wait_for_service(timeout_sec=2.0):
            return False, 'Failed: /slam_toolbox/serialize_map service is not available.'

        posegraph_path = os.path.join(self.posegraph_save_dir, safe_name)

        req = SerializePoseGraph.Request()
        req.filename = posegraph_path

        future = self.serialize_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            return False, 'Failed: serialize posegraph service call failed.'

        return True, f'PoseGraph serialized: {posegraph_path}'

    def handle_save_map_name(self, msg):
        success, message = self.save_map_with_name(msg.data)

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

        self.publish_event('save_map', success, message)

    def handle_serialize_posegraph_name(self, msg):
        success, message = self.serialize_posegraph_with_name(msg.data)

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

        self.publish_event('serialize_posegraph', success, message)

    def handle_reset_slam(self, request, response):
        if not self.reset_client.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = 'Failed: /slam_toolbox/reset service is not available.'
            self.publish_event('reset_slam', False, response.message)
            return response

        future = self.reset_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            response.success = False
            response.message = 'Failed: SLAM reset service call failed.'
            self.publish_event('reset_slam', False, response.message)
            return response

        response.success = True
        response.message = (
            'SLAM reset service called. '
            'Full empty-map initialization may require restarting slam_toolbox.'
        )

        self.get_logger().info(response.message)
        self.publish_event('reset_slam', True, response.message)
        return response

    def handle_save_map_auto(self, request, response):
        map_name = self.get_timestamp_name('slam_map')
        success, message = self.save_map_with_name(map_name)

        response.success = success
        response.message = message

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

        self.publish_event('save_map', success, message)
        return response

    def handle_serialize_posegraph_auto(self, request, response):
        posegraph_name = self.get_timestamp_name('slam_posegraph')
        success, message = self.serialize_posegraph_with_name(posegraph_name)

        response.success = success
        response.message = message

        if success:
            self.get_logger().info(message)
        else:
            self.get_logger().error(message)

        self.publish_event('serialize_posegraph', success, message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SlamManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
