import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Empty
from std_msgs.msg import String
from std_msgs.msg import Int32

from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import FollowWaypoints

from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalInfo


class WebGoalManager(Node):

    def __init__(self):
        super().__init__('web_goal_manager')

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.waypoint_client = ActionClient(
            self,
            FollowWaypoints,
            '/follow_waypoints'
        )

        self.navigate_cancel_client = self.create_client(
            CancelGoal,
            '/navigate_to_pose/_action/cancel_goal'
        )

        self.waypoint_cancel_client = self.create_client(
            CancelGoal,
            '/follow_waypoints/_action/cancel_goal'
        )

        self.current_goal_handle = None
        self.current_waypoint_handle = None

        self.waypoints = []
        self.home_pose = None

        self.goal_status_pub = self.create_publisher(
            String,
            '/web_goal_status',
            10
        )

        self.waypoint_count_pub = self.create_publisher(
            Int32,
            '/web_waypoint_count',
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/web_goal_pose',
            self.goal_callback,
            10
        )

        self.cancel_sub = self.create_subscription(
            Empty,
            '/web_cancel_goal',
            self.cancel_callback,
            10
        )

        self.waypoint_add_sub = self.create_subscription(
            PoseStamped,
            '/web_waypoint_add',
            self.waypoint_add_callback,
            10
        )

        self.waypoint_start_sub = self.create_subscription(
            Empty,
            '/web_waypoint_start',
            self.waypoint_start_callback,
            10
        )

        self.waypoint_clear_sub = self.create_subscription(
            Empty,
            '/web_waypoint_clear',
            self.waypoint_clear_callback,
            10
        )

        self.set_home_sub = self.create_subscription(
            PoseStamped,
            '/web_set_home_pose',
            self.set_home_callback,
            10
        )

        self.return_home_sub = self.create_subscription(
            Empty,
            '/web_return_home',
            self.return_home_callback,
            10
        )

        self.publish_goal_status('IDLE')
        self.publish_waypoint_count()

        self.get_logger().info(
            'Web Goal Manager started | '
            'Goal=/web_goal_pose | '
            'Cancel=/web_cancel_goal | '
            'Waypoint Add=/web_waypoint_add | '
            'Waypoint Start=/web_waypoint_start | '
            'Waypoint Clear=/web_waypoint_clear | '
            'Set Home=/web_set_home_pose | '
            'Return Home=/web_return_home'
        )

    def publish_goal_status(self, status):
        msg = String()
        msg.data = status
        self.goal_status_pub.publish(msg)

    def publish_waypoint_count(self):
        msg = Int32()
        msg.data = len(self.waypoints)
        self.waypoint_count_pub.publish(msg)

    def stamp_pose(self, pose_msg):
        pose_msg.header.stamp = self.get_clock().now().to_msg()

        if pose_msg.header.frame_id == '':
            pose_msg.header.frame_id = 'map'

        return pose_msg

    def cancel_all_action_goals(self):
        cancel_request = CancelGoal.Request()
        cancel_request.goal_info = GoalInfo()

        nav_sent = False
        waypoint_sent = False

        if self.navigate_cancel_client.wait_for_service(timeout_sec=0.5):
            future = self.navigate_cancel_client.call_async(cancel_request)
            future.add_done_callback(self.cancel_service_done_callback)
            nav_sent = True
        else:
            self.get_logger().warn(
                'NavigateToPose cancel service not available'
            )

        if self.waypoint_cancel_client.wait_for_service(timeout_sec=0.5):
            future = self.waypoint_cancel_client.call_async(cancel_request)
            future.add_done_callback(self.cancel_service_done_callback)
            waypoint_sent = True
        else:
            self.get_logger().warn(
                'FollowWaypoints cancel service not available'
            )

        return nav_sent or waypoint_sent

    def cancel_service_done_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                f'Cancel service response received | '
                f'goals_canceling={len(response.goals_canceling)}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Cancel service call failed: {str(e)}'
            )

    def send_navigate_goal(self, pose_msg, status_prefix='SENDING_GOAL'):
        self.cancel_all_action_goals()

        self.current_goal_handle = None
        self.current_waypoint_handle = None

        if not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                'NavigateToPose action server not available'
            )
            self.publish_goal_status('NAV_SERVER_NOT_AVAILABLE')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.stamp_pose(pose_msg)

        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            self.navigate_goal_response_callback
        )

        self.publish_goal_status(status_prefix)

    def goal_callback(self, msg):
        self.get_logger().info(
            f'Received WEB goal: x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )

        self.send_navigate_goal(msg, 'SENDING_GOAL')

    def navigate_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Navigate goal rejected')
            self.publish_goal_status('REJECTED')
            return

        self.current_goal_handle = goal_handle

        self.get_logger().info('Navigate goal accepted')
        self.publish_goal_status('MOVING')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.navigate_result_callback
        )

    def navigate_result_callback(self, future):
        result = future.result()

        status = result.status
        self.current_goal_handle = None

        if status == 4:
            self.get_logger().info('Navigate goal succeeded')
            self.publish_goal_status('ARRIVED')
        else:
            self.get_logger().warn(
                f'Navigate goal finished with status: {status}'
            )
            self.publish_goal_status(f'FINISHED_{status}')

    def cancel_callback(self, msg):
        self.get_logger().info(
            'Cancel request received from WEB | cancel all goals'
        )

        self.cancel_all_action_goals()

        if self.current_goal_handle is not None:
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    self.cancel_handle_done_callback
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Current navigate goal handle cancel failed: {str(e)}'
                )

        if self.current_waypoint_handle is not None:
            try:
                cancel_future = self.current_waypoint_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    self.cancel_handle_done_callback
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Current waypoint goal handle cancel failed: {str(e)}'
                )

        self.current_goal_handle = None
        self.current_waypoint_handle = None

        self.publish_goal_status('CANCELED')

    def cancel_handle_done_callback(self, future):
        try:
            future.result()
            self.get_logger().info('Goal handle cancel completed')
        except Exception as e:
            self.get_logger().error(
                f'Goal handle cancel failed: {str(e)}'
            )

    def waypoint_add_callback(self, msg):
        pose = self.stamp_pose(msg)
        self.waypoints.append(pose)

        self.get_logger().info(
            f'Waypoint added: #{len(self.waypoints)} '
            f'x={pose.pose.position.x:.2f}, '
            f'y={pose.pose.position.y:.2f}'
        )

        self.publish_waypoint_count()

    def waypoint_clear_callback(self, msg):
        self.cancel_all_action_goals()

        self.current_goal_handle = None
        self.current_waypoint_handle = None

        self.waypoints = []

        self.get_logger().info('Waypoint queue cleared')
        self.publish_waypoint_count()
        self.publish_goal_status('WAYPOINT_CLEARED')

    def waypoint_start_callback(self, msg):
        if len(self.waypoints) == 0:
            self.get_logger().warn('No waypoints to start')
            self.publish_goal_status('NO_WAYPOINTS')
            return

        self.cancel_all_action_goals()

        self.current_goal_handle = None
        self.current_waypoint_handle = None

        if not self.waypoint_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(
                'FollowWaypoints action server not available'
            )
            self.publish_goal_status('WAYPOINT_SERVER_NOT_AVAILABLE')
            return

        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.waypoints

        send_goal_future = self.waypoint_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            self.waypoint_goal_response_callback
        )

        self.publish_goal_status('SENDING_WAYPOINTS')

    def waypoint_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('Waypoint goal rejected')
            self.publish_goal_status('WAYPOINT_REJECTED')
            return

        self.current_waypoint_handle = goal_handle

        self.get_logger().info('Waypoint goal accepted')
        self.publish_goal_status('WAYPOINT_RUNNING')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.waypoint_result_callback
        )

    def waypoint_result_callback(self, future):
        result = future.result()

        status = result.status
        self.current_waypoint_handle = None

        if status == 4:
            self.get_logger().info('Waypoint patrol succeeded')
            self.publish_goal_status('WAYPOINT_DONE')
        else:
            self.get_logger().warn(
                f'Waypoint patrol finished with status: {status}'
            )
            self.publish_goal_status(f'WAYPOINT_FINISHED_{status}')

    def set_home_callback(self, msg):
        self.home_pose = self.stamp_pose(msg)

        self.get_logger().info(
            f'Home pose saved: x={self.home_pose.pose.position.x:.2f}, '
            f'y={self.home_pose.pose.position.y:.2f}'
        )

        self.publish_goal_status('HOME_SET')

    def return_home_callback(self, msg):
        if self.home_pose is None:
            self.get_logger().warn(
                'Return Home requested, but home pose is not set'
            )
            self.publish_goal_status('NO_HOME_POSE')
            return

        self.get_logger().info(
            f'Return Home requested: x={self.home_pose.pose.position.x:.2f}, '
            f'y={self.home_pose.pose.position.y:.2f}'
        )

        self.send_navigate_goal(self.home_pose, 'RETURN_HOME')


def main(args=None):
    rclpy.init(args=args)

    node = WebGoalManager()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
