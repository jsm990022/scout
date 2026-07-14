#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints


class MultiGoalMissionNode(Node):
    def __init__(self):
        super().__init__('multi_goal_mission_node')

        self.client = ActionClient(
            self,
            FollowWaypoints,
            'follow_waypoints'
        )

        self.waypoints = [
            self.create_pose_quat(0.9415, 5.1834, 0.9219, 0.3875),
            self.create_pose_quat(-8.8748, 14.5104, 0.9426, 0.3339),
            self.create_pose_quat(-18.7762, 4.4871, -0.9239, 0.3826),
            self.create_pose_quat(-27.8812, -4.6316, -0.9243, 0.3817),
            self.create_pose_quat(-9.3806, 14.2112, 0.4041, 0.9147),
            self.create_pose_quat(6.9967, -0.4487, -0.2707, 0.9626),
            self.create_pose_quat(21.7283, -14.9112, -0.3220, 0.9468),
            self.create_pose_quat(13.6023, -7.0807, 0.9248, 0.3803),
            self.create_pose_quat(3.4271, 2.9169, -0.9138, 0.4063),
        ]

        self.get_logger().info('Waiting for /follow_waypoints action server...')
        self.client.wait_for_server()

        self.get_logger().info('Action server connected. Sending multi-goal mission...')
        self.send_goal()

    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        return pose

    def create_pose_quat(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w

        return pose

    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Multi-goal mission rejected.')
            return

        self.get_logger().info('Multi-goal mission accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        current_waypoint = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Current waypoint index: {current_waypoint}')

    def result_callback(self, future):
        result = future.result().result
        missed = result.missed_waypoints

        if len(missed) == 0:
            self.get_logger().info('Multi-goal mission completed successfully.')
        else:
            self.get_logger().warn(f'Missed waypoints: {missed}')


def main(args=None):
    rclpy.init(args=args)
    node = MultiGoalMissionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
