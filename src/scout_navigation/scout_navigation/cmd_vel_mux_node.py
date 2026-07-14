import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from std_msgs.msg import String


class CmdVelMuxNode(Node):

    def __init__(self):
        super().__init__('cmd_vel_mux_node')

        self.current_mode = 'manual'
        self.recovery_active = False

        self.latest_nav_cmd = Twist()
        self.latest_web_cmd = Twist()
        self.latest_recovery_cmd = Twist()

        self.nav_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.nav_cmd_callback,
            10
        )

        self.web_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_web',
            self.web_cmd_callback,
            10
        )

        self.recovery_cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel_recovery',
            self.recovery_cmd_callback,
            10
        )

        self.mode_sub = self.create_subscription(
            String,
            '/web_mode',
            self.mode_callback,
            10
        )

        self.recovery_active_sub = self.create_subscription(
            Bool,
            '/recovery_active',
            self.recovery_active_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_selected',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.timer_callback
        )

        self.get_logger().info(
            'Cmd Vel Mux Active | '
            'manual=/cmd_vel_web | '
            'auto=/cmd_vel_nav | '
            'recovery=/cmd_vel_recovery | '
            'output=/cmd_vel_selected'
        )

    def nav_cmd_callback(self, msg):
        self.latest_nav_cmd = msg

    def web_cmd_callback(self, msg):
        self.latest_web_cmd = msg

    def recovery_cmd_callback(self, msg):
        self.latest_recovery_cmd = msg

    def recovery_active_callback(self, msg):
        self.recovery_active = msg.data

    def mode_callback(self, msg):
        if msg.data not in ['manual', 'auto']:
            self.get_logger().warn(
                f'Invalid web mode received: {msg.data}'
            )
            return

        if self.current_mode != msg.data:
            self.current_mode = msg.data

            stop_msg = Twist()
            self.cmd_pub.publish(stop_msg)

            self.get_logger().info(
                f'Control mode changed: {self.current_mode}'
            )

    def timer_callback(self):
        if self.recovery_active:
            selected_cmd = self.latest_recovery_cmd
        elif self.current_mode == 'manual':
            selected_cmd = self.latest_web_cmd
        else:
            selected_cmd = self.latest_nav_cmd

        self.cmd_pub.publish(selected_cmd)


def main(args=None):
    rclpy.init(args=args)

    node = CmdVelMuxNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()