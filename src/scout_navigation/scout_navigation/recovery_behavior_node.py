#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy


class RecoveryBehaviorNode(Node):

    def __init__(self):
        super().__init__('recovery_behavior_node')

        # Safety Stop 기준 거리
        self.front_stop_distance = 0.90

        # Recovery 종료 기준 거리
        # 0.9m보다 조금 더 크게 잡아서 충분히 열렸을 때만 종료
        self.front_clear_distance = 1.10

        # 좌/우/후방 공간 판단 기준
        self.side_clear_distance = 0.80
        self.back_clear_distance = 0.80

        # 전방이 막힌 상태가 이 시간 이상 유지되면 recovery 시작
        self.stuck_time_threshold = 3.0

        # 너무 오래 회전하지 않도록 최대 recovery 시간 제한
        self.max_recovery_duration = 6.0

        # recovery 명령
        self.turn_speed = 0.35
        self.back_speed = -0.06

        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')
        self.back_distance = float('inf')

        self.last_clear_time = time.time()

        self.is_recovering = False
        self.recovery_start_time = 0.0
        self.recovery_mode = 'WAIT'
        self.recovery_cmd = Twist()

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

        self.recovery_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel_recovery',
            10
        )

        self.recovery_active_pub = self.create_publisher(
            Bool,
            '/recovery_active',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )

        self.get_logger().info(
            'Recovery Behavior Node Active | '
            'turn until front is clear | '
            'output=/cmd_vel_recovery | status=/recovery_active'
        )

    def get_sector_min(self, msg, center_deg, half_width_deg):
        center_rad = math.radians(center_deg)
        half_width_rad = math.radians(half_width_deg)

        start_angle = center_rad - half_width_rad
        end_angle = center_rad + half_width_rad

        values = []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r <= 0.0:
                continue

            angle = msg.angle_min + i * msg.angle_increment

            if start_angle <= angle <= end_angle:
                values.append(r)

        if not values:
            return float('inf')

        return min(values)

    def scan_callback(self, msg):
        self.front_distance = self.get_sector_min(msg, 0.0, 30.0)
        self.left_distance = self.get_sector_min(msg, 90.0, 35.0)
        self.right_distance = self.get_sector_min(msg, -90.0, 35.0)
        self.back_distance = self.get_sector_min(msg, 180.0, 35.0)

        if self.front_distance >= self.front_stop_distance:
            self.last_clear_time = time.time()

    def publish_recovery_state(self, active):
        msg = Bool()
        msg.data = active
        self.recovery_active_pub.publish(msg)

    def start_recovery(self):
        self.recovery_cmd = Twist()

        # 좌/우 중 더 넓은 쪽을 우선 선택
        if self.left_distance > self.side_clear_distance or self.right_distance > self.side_clear_distance:

            if self.left_distance >= self.right_distance:
                self.recovery_mode = 'TURN_LEFT'
                self.recovery_cmd.angular.z = self.turn_speed
                self.get_logger().warn(
                    f'Recovery START: TURN_LEFT | '
                    f'front={self.front_distance:.2f}m, '
                    f'left={self.left_distance:.2f}m, '
                    f'right={self.right_distance:.2f}m'
                )
            else:
                self.recovery_mode = 'TURN_RIGHT'
                self.recovery_cmd.angular.z = -self.turn_speed
                self.get_logger().warn(
                    f'Recovery START: TURN_RIGHT | '
                    f'front={self.front_distance:.2f}m, '
                    f'left={self.left_distance:.2f}m, '
                    f'right={self.right_distance:.2f}m'
                )

        elif self.back_distance > self.back_clear_distance:
            self.recovery_mode = 'BACK'
            self.recovery_cmd.linear.x = self.back_speed
            self.get_logger().warn(
                f'Recovery START: BACK | '
                f'back={self.back_distance:.2f}m'
            )

        else:
            self.recovery_mode = 'WAIT'
            self.recovery_cmd = Twist()
            self.get_logger().warn(
                'Recovery START: WAIT | all directions blocked'
            )

        self.is_recovering = True
        self.recovery_start_time = time.time()
        self.publish_recovery_state(True)
        self.recovery_cmd_pub.publish(self.recovery_cmd)

    def stop_recovery(self, reason):
        self.is_recovering = False
        self.recovery_mode = 'WAIT'
        self.recovery_cmd = Twist()

        self.publish_recovery_state(False)
        self.recovery_cmd_pub.publish(Twist())

        self.last_clear_time = time.time()

        self.get_logger().info(
            f'Recovery FINISHED | reason={reason} | '
            f'front={self.front_distance:.2f}m'
        )

    def timer_callback(self):
        now = time.time()

        if self.is_recovering:

            recovery_time = now - self.recovery_start_time

            # 전방이 충분히 열리면 recovery 종료
            if self.front_distance >= self.front_clear_distance:
                self.stop_recovery('front_clear')
                return

            # 너무 오래 recovery하면 대기 상태로 종료
            if recovery_time >= self.max_recovery_duration:
                self.stop_recovery('timeout')
                return

            # recovery 중에는 선택한 명령을 계속 발행
            self.publish_recovery_state(True)
            self.recovery_cmd_pub.publish(self.recovery_cmd)
            return

        front_blocked = self.front_distance < self.front_stop_distance
        stuck_duration = now - self.last_clear_time

        if front_blocked and stuck_duration > self.stuck_time_threshold:
            self.start_recovery()
        else:
            self.publish_recovery_state(False)


def main(args=None):
    rclpy.init(args=args)

    node = RecoveryBehaviorNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
