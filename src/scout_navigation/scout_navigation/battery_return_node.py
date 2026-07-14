import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from scout_msgs.msg import ScoutStatus


class BatteryReturnNode(Node):
    def __init__(self):
        super().__init__('battery_return_node')

        self.declare_parameter('low_battery_voltage', 25.5)

        self.low_battery_voltage = (
            self.get_parameter('low_battery_voltage')
            .get_parameter_value()
            .double_value
        )

        self.return_triggered = False

        self.client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.sub = self.create_subscription(
            ScoutStatus,
            '/scout_status',
            self.status_callback,
            10
        )

        self.get_logger().info(
            f'Battery Return Node Active | threshold={self.low_battery_voltage:.2f}V'
        )

    def status_callback(self, msg):
        voltage = msg.battery_voltage

        self.get_logger().info(
            f'Battery voltage: {voltage:.2f}V'
        )

        if self.return_triggered:
            return

        if voltage <= self.low_battery_voltage:
            self.return_triggered = True
            self.get_logger().warn(
                f'Low battery detected: {voltage:.2f}V <= {self.low_battery_voltage:.2f}V'
            )
            self.send_return_goal()

    def send_return_goal(self):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        self.client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.create_charging_pose()

        self.get_logger().warn('Sending robot to charging station...')

        send_goal_future = self.client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def create_charging_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = 2.9942
        pose.pose.position.y = 1.6546
        pose.pose.position.z = 0.0

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.4053
        pose.pose.orientation.w = 0.9142

        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Battery return goal rejected.')
            self.return_triggered = False
            return

        self.get_logger().info('Battery return goal accepted.')

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining

        self.get_logger().info(
            f'Returning to charging station... distance remaining: {distance:.2f} m'
        )

    def result_callback(self, future):
        status = future.result().status

        if status == 4:
            self.get_logger().info('Arrived at charging station.')
        else:
            self.get_logger().warn(
                f'Battery return finished with status: {status}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BatteryReturnNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
