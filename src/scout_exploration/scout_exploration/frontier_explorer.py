#!/usr/bin/env python3

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped

from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import ComputePathToPose, NavigateToPose


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        self.declare_parameter('force_no_frontier_test', False)
        self.declare_parameter('min_cluster_size', 8)
        self.declare_parameter('min_target_distance', 0.5)
        self.declare_parameter('blacklist_radius', 1.0)
        self.declare_parameter('max_navigation_time', 180.0)
        self.declare_parameter('progress_check_window', 45.0)
        self.declare_parameter('minimum_progress', 0.3)
        self.declare_parameter('next_goal_delay_sec', 3.0)

        self.force_no_frontier_test = self.get_parameter('force_no_frontier_test').value
        self.min_cluster_size = self.get_parameter('min_cluster_size').value
        self.min_target_distance = self.get_parameter('min_target_distance').value
        self.blacklist_radius = self.get_parameter('blacklist_radius').value
        self.max_navigation_time = self.get_parameter('max_navigation_time').value
        self.progress_check_window = self.get_parameter('progress_check_window').value
        self.minimum_progress = self.get_parameter('minimum_progress').value
        self.next_goal_delay_sec = self.get_parameter('next_goal_delay_sec').value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/frontier_markers',
            10
        )

        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/frontier_goal_pose',
            10
        )

        self.validated_path_pub = self.create_publisher(
            Path,
            '/frontier_validated_path',
            10
        )

        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            '/compute_path_to_pose'
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            '/navigate_to_pose'
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.blacklist = []

        self.last_checked_goal_x = None
        self.last_checked_goal_y = None

        self.path_check_in_progress = False
        self.navigation_goal_sent = False
        self.navigation_in_progress = False
        self.waiting_for_next_goal = False
        self.exploration_completed = False
        self.failure_handling = False

        self.pending_nav_goal = None
        self.current_nav_goal = None
        self.nav_goal_handle = None

        self.navigation_start_time = None
        self.progress_window_start_time = None
        self.progress_window_start_distance = None
        self.latest_distance_remaining = None

        self.reset_timer = None

        self.watchdog_timer = self.create_timer(
            1.0,
            self.navigation_watchdog
        )

        self.get_logger().warn(
            'Frontier Explorer started. '
            'Auto exploration + blacklist + timeout + progress check enabled.'
        )

        self.get_logger().info(
            f'Parameters: '
            f'min_cluster_size={self.min_cluster_size}, '
            f'min_target_distance={self.min_target_distance}, '
            f'blacklist_radius={self.blacklist_radius}, '
            f'max_navigation_time={self.max_navigation_time}, '
            f'progress_check_window={self.progress_check_window}, '
            f'minimum_progress={self.minimum_progress}, '
            f'next_goal_delay_sec={self.next_goal_delay_sec}'
        )

        if self.force_no_frontier_test:
            self.get_logger().warn(
                'Force no frontier test mode is ON.'
            )

    def map_callback(self, msg):
        if (
            self.exploration_completed or
            self.navigation_goal_sent or
            self.navigation_in_progress or
            self.waiting_for_next_goal or
            self.failure_handling
        ):
            return

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = msg.data

        frontier_cells = []

        if not self.force_no_frontier_test:
            for y in range(1, height - 1):
                for x in range(1, width - 1):
                    index = y * width + x

                    if data[index] == 0 and self.has_unknown_neighbor(
                        data,
                        x,
                        y,
                        width
                    ):
                        frontier_cells.append((x, y))
        else:
            self.get_logger().warn(
                'Force no frontier test mode active. '
                'frontier_cells intentionally set to empty.'
            )

        clusters = self.cluster_frontiers(frontier_cells, width, height)

        cluster_centers = self.calculate_cluster_centers(
            clusters,
            resolution,
            origin_x,
            origin_y
        )

        frontier_points = [
            self.cell_to_world(x, y, resolution, origin_x, origin_y)
            for x, y in frontier_cells
        ]

        if not cluster_centers:
            self.publish_frontier_markers(
                frontier_points,
                cluster_centers,
                msg.header.frame_id,
                None
            )

            self.finish_exploration(
                len(frontier_cells),
                len(cluster_centers)
            )
            return

        robot_position = self.get_robot_position()

        if robot_position is None:
            self.get_logger().warn(
                'Robot position not found yet. Check TF: map -> base_footprint'
            )
            return

        robot_x, robot_y = robot_position

        best_frontier = self.find_best_exploration_target(
            cluster_centers,
            robot_x,
            robot_y
        )

        self.publish_frontier_markers(
            frontier_points,
            cluster_centers,
            msg.header.frame_id,
            best_frontier
        )

        if best_frontier is None:
            self.finish_exploration(
                len(frontier_cells),
                len(cluster_centers)
            )
            return

        best_x, best_y, best_distance, best_score, best_size = best_frontier

        goal_pose = self.create_goal_pose(
            best_x,
            best_y,
            msg.header.frame_id,
            robot_x,
            robot_y
        )

        self.goal_pose_pub.publish(goal_pose)

        self.get_logger().info(
            f'Frontier Cells: {len(frontier_cells)} | '
            f'Clusters: {len(cluster_centers)} | '
            f'Blacklist: {len(self.blacklist)} | '
            f'Navigation Candidate: x={best_x:.2f}, '
            f'y={best_y:.2f}, '
            f'distance={best_distance:.2f}, '
            f'size={best_size}, '
            f'score={best_score:.2f}'
        )

        self.request_path_check_if_needed(goal_pose)

    def finish_exploration(self, frontier_count, cluster_count):
        self.exploration_completed = True

        self.get_logger().warn(
            'Exploration completed. '
            'No selectable frontier remains. '
            f'Frontier Cells={frontier_count}, '
            f'Clusters={cluster_count}, '
            f'Blacklist={len(self.blacklist)}'
        )

    def has_unknown_neighbor(self, data, x, y, width):
        for dy in [-1, 0, 1]:
            for dx in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue

                neighbor_index = (y + dy) * width + (x + dx)

                if data[neighbor_index] == -1:
                    return True

        return False

    def cluster_frontiers(self, frontier_cells, width, height):
        frontier_set = set(frontier_cells)
        visited = set()
        clusters = []

        for cell in frontier_cells:
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)

            while queue:
                current = queue.popleft()
                cluster.append(current)

                cx, cy = current

                for dy in [-1, 0, 1]:
                    for dx in [-1, 0, 1]:
                        if dx == 0 and dy == 0:
                            continue

                        nx = cx + dx
                        ny = cy + dy
                        neighbor = (nx, ny)

                        if (
                            0 <= nx < width and
                            0 <= ny < height and
                            neighbor in frontier_set and
                            neighbor not in visited
                        ):
                            visited.add(neighbor)
                            queue.append(neighbor)

            if len(cluster) >= self.min_cluster_size:
                clusters.append(cluster)

        return clusters

    def calculate_cluster_centers(self, clusters, resolution, origin_x, origin_y):
        centers = []

        for cluster in clusters:
            sum_x = 0.0
            sum_y = 0.0

            for cell_x, cell_y in cluster:
                world_x, world_y = self.cell_to_world(
                    cell_x,
                    cell_y,
                    resolution,
                    origin_x,
                    origin_y
                )

                sum_x += world_x
                sum_y += world_y

            center_x = sum_x / len(cluster)
            center_y = sum_y / len(cluster)

            centers.append((center_x, center_y, len(cluster)))

        return centers

    def cell_to_world(self, x, y, resolution, origin_x, origin_y):
        world_x = origin_x + (x + 0.5) * resolution
        world_y = origin_y + (y + 0.5) * resolution

        return world_x, world_y

    def get_robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except Exception:
            return None

    def find_best_exploration_target(self, cluster_centers, robot_x, robot_y):
        if not cluster_centers:
            return None

        best_x = None
        best_y = None
        best_distance = float('inf')
        best_score = -1.0
        best_size = 0

        for center_x, center_y, cluster_size in cluster_centers:
            if self.is_blacklisted(center_x, center_y):
                continue

            distance = math.sqrt(
                (center_x - robot_x) ** 2 +
                (center_y - robot_y) ** 2
            )

            if distance < self.min_target_distance:
                continue

            score = cluster_size / max(distance, 0.001)

            if score > best_score:
                best_score = score
                best_distance = distance
                best_x = center_x
                best_y = center_y
                best_size = cluster_size

        if best_x is None:
            return None

        return best_x, best_y, best_distance, best_score, best_size

    def is_blacklisted(self, x, y):
        for bx, by in self.blacklist:
            distance = math.sqrt((x - bx) ** 2 + (y - by) ** 2)

            if distance < self.blacklist_radius:
                return True

        return False

    def add_to_blacklist(self, goal_pose, reason):
        if goal_pose is None:
            return

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        if not self.is_blacklisted(x, y):
            self.blacklist.append((x, y))

        self.get_logger().error(
            f'Added frontier to blacklist: '
            f'x={x:.2f}, y={y:.2f}, '
            f'reason={reason}, '
            f'blacklist_count={len(self.blacklist)}'
        )

    def create_goal_pose(self, x, y, frame_id, robot_x, robot_y):
        goal = PoseStamped()
        goal.header.frame_id = frame_id
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        yaw = math.atan2(y - robot_y, x - robot_x)
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        return goal

    def request_path_check_if_needed(self, goal_pose):
        if self.path_check_in_progress:
            return

        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y

        if self.last_checked_goal_x is not None and self.last_checked_goal_y is not None:
            moved = math.sqrt(
                (goal_x - self.last_checked_goal_x) ** 2 +
                (goal_y - self.last_checked_goal_y) ** 2
            )

            if moved < 0.3:
                return

        if not self.compute_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                'ComputePathToPose action server not available yet.'
            )
            return

        self.last_checked_goal_x = goal_x
        self.last_checked_goal_y = goal_y
        self.path_check_in_progress = True
        self.pending_nav_goal = goal_pose

        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal = goal_pose
        goal_msg.use_start = False
        goal_msg.planner_id = ''

        self.get_logger().warn(
            f'Requesting path validation before movement: '
            f'x={goal_x:.2f}, y={goal_y:.2f}'
        )

        future = self.compute_path_client.send_goal_async(goal_msg)
        future.add_done_callback(self.path_goal_response_callback)

    def path_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Path validation goal was rejected by planner_server.'
            )

            self.add_to_blacklist(
                self.pending_nav_goal,
                'planner_goal_rejected'
            )

            self.path_check_in_progress = False
            self.pending_nav_goal = None
            self.schedule_next_frontier()
            return

        self.get_logger().info(
            'Path validation goal accepted by planner_server.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)

    def path_result_callback(self, future):
        self.path_check_in_progress = False

        result_msg = future.result().result
        status = future.result().status

        path = result_msg.path
        pose_count = len(path.poses)

        if status == 4 and pose_count > 0:
            self.validated_path_pub.publish(path)
            self.get_logger().warn(
                f'Path validation SUCCESS. '
                f'Path poses={pose_count}. '
                f'Now sending this frontier to Nav2.'
            )

            if self.pending_nav_goal is not None:
                self.send_navigation_goal(self.pending_nav_goal)
        else:
            self.get_logger().error(
                f'Path validation FAILED. '
                f'status={status}, path poses={pose_count}.'
            )

            self.add_to_blacklist(
                self.pending_nav_goal,
                'planner_failed'
            )

            self.pending_nav_goal = None
            self.schedule_next_frontier()

    def send_navigation_goal(self, goal_pose):
        if self.navigation_goal_sent or self.navigation_in_progress:
            return

        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                'NavigateToPose action server not available. Robot will not move.'
            )

            self.add_to_blacklist(
                goal_pose,
                'navigate_action_server_unavailable'
            )

            self.schedule_next_frontier()
            return

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_pose

        self.navigation_goal_sent = True
        self.navigation_in_progress = True
        self.current_nav_goal = goal_pose
        self.pending_nav_goal = None

        self.navigation_start_time = self.get_time_sec()
        self.progress_window_start_time = self.navigation_start_time
        self.progress_window_start_distance = None
        self.latest_distance_remaining = None

        self.get_logger().warn(
            f'Sending validated Frontier goal to Nav2: '
            f'x={goal_pose.pose.position.x:.2f}, '
            f'y={goal_pose.pose.position.y:.2f}'
        )

        future = self.navigate_client.send_goal_async(
            nav_goal,
            feedback_callback=self.navigation_feedback_callback
        )
        future.add_done_callback(self.navigation_goal_response_callback)

    def navigation_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                'Validated Frontier goal was rejected by Nav2.'
            )

            self.add_to_blacklist(
                self.current_nav_goal,
                'navigate_goal_rejected'
            )

            self.navigation_goal_sent = False
            self.navigation_in_progress = False
            self.current_nav_goal = None
            self.nav_goal_handle = None
            self.schedule_next_frontier()
            return

        self.nav_goal_handle = goal_handle

        self.get_logger().info(
            'Validated Frontier goal accepted by Nav2.'
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result_callback)

    def navigation_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        distance = feedback.distance_remaining
        self.latest_distance_remaining = distance

        if self.progress_window_start_distance is None:
            self.progress_window_start_distance = distance
            self.progress_window_start_time = self.get_time_sec()

        self.get_logger().info(
            f'Navigation feedback: '
            f'distance_remaining={distance:.2f}'
        )

    def navigation_watchdog(self):
        if not self.navigation_in_progress:
            return

        if self.failure_handling:
            return

        now = self.get_time_sec()

        if self.navigation_start_time is not None:
            elapsed = now - self.navigation_start_time

            if elapsed > self.max_navigation_time:
                self.handle_navigation_failure(
                    f'timeout_over_{self.max_navigation_time:.0f}s'
                )
                return

        if (
            self.progress_window_start_time is not None and
            self.progress_window_start_distance is not None and
            self.latest_distance_remaining is not None
        ):
            window_elapsed = now - self.progress_window_start_time

            if window_elapsed >= self.progress_check_window:
                progress = (
                    self.progress_window_start_distance -
                    self.latest_distance_remaining
                )

                if progress < self.minimum_progress:
                    self.handle_navigation_failure(
                        f'insufficient_progress_{progress:.2f}m'
                    )
                    return

                self.progress_window_start_time = now
                self.progress_window_start_distance = self.latest_distance_remaining

    def handle_navigation_failure(self, reason):
        if self.failure_handling:
            return

        self.failure_handling = True

        self.get_logger().error(
            f'Navigation failure detected: {reason}. '
            'Canceling current goal and blacklisting frontier.'
        )

        self.add_to_blacklist(self.current_nav_goal, reason)

        if self.nav_goal_handle is not None:
            cancel_future = self.nav_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.finish_failure_handling()

    def cancel_done_callback(self, future):
        self.get_logger().warn(
            'Current Nav2 goal cancel request completed.'
        )
        self.finish_failure_handling()

    def finish_failure_handling(self):
        self.navigation_goal_sent = False
        self.navigation_in_progress = False
        self.path_check_in_progress = False
        self.failure_handling = False

        self.pending_nav_goal = None
        self.current_nav_goal = None
        self.nav_goal_handle = None

        self.clear_navigation_monitoring()
        self.schedule_next_frontier()

    def navigation_result_callback(self, future):
        if self.failure_handling:
            return

        self.navigation_in_progress = False
        self.navigation_goal_sent = False

        status = future.result().status

        if status == 4:
            self.get_logger().warn(
                'Frontier navigation SUCCEEDED. '
                f'Waiting {self.next_goal_delay_sec:.1f} seconds before next frontier.'
            )

            self.current_nav_goal = None
            self.nav_goal_handle = None
            self.clear_navigation_monitoring()
            self.schedule_next_frontier()
        else:
            self.get_logger().error(
                f'Frontier navigation ended with status={status}. '
                'This frontier will be blacklisted.'
            )

            self.add_to_blacklist(
                self.current_nav_goal,
                f'navigation_result_status_{status}'
            )

            self.current_nav_goal = None
            self.nav_goal_handle = None
            self.clear_navigation_monitoring()
            self.schedule_next_frontier()

    def clear_navigation_monitoring(self):
        self.navigation_start_time = None
        self.progress_window_start_time = None
        self.progress_window_start_distance = None
        self.latest_distance_remaining = None

    def schedule_next_frontier(self):
        if self.exploration_completed:
            return

        self.waiting_for_next_goal = True
        self.start_reset_timer()

    def start_reset_timer(self):
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None

        self.reset_timer = self.create_timer(
            self.next_goal_delay_sec,
            self.reset_for_next_frontier
        )

    def reset_for_next_frontier(self):
        if self.reset_timer is not None:
            self.reset_timer.cancel()
            self.destroy_timer(self.reset_timer)
            self.reset_timer = None

        self.navigation_goal_sent = False
        self.navigation_in_progress = False
        self.path_check_in_progress = False
        self.waiting_for_next_goal = False
        self.failure_handling = False
        self.pending_nav_goal = None

        self.last_checked_goal_x = None
        self.last_checked_goal_y = None

        self.get_logger().warn(
            'Ready for next frontier. Recomputing from latest /map...'
        )

    def get_time_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def publish_frontier_markers(
        self,
        frontier_points,
        cluster_centers,
        frame_id,
        best_frontier
    ):
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.header.frame_id = frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.ns = 'frontiers'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        frontier_marker = Marker()
        frontier_marker.header.frame_id = frame_id
        frontier_marker.header.stamp = self.get_clock().now().to_msg()
        frontier_marker.ns = 'frontier_cells'
        frontier_marker.id = 1
        frontier_marker.type = Marker.POINTS
        frontier_marker.action = Marker.ADD
        frontier_marker.scale.x = 0.06
        frontier_marker.scale.y = 0.06
        frontier_marker.color.r = 0.0
        frontier_marker.color.g = 1.0
        frontier_marker.color.b = 0.0
        frontier_marker.color.a = 1.0

        for x, y in frontier_points:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.05
            frontier_marker.points.append(p)

        marker_array.markers.append(frontier_marker)

        center_marker = Marker()
        center_marker.header.frame_id = frame_id
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = 'frontier_centers'
        center_marker.id = 2
        center_marker.type = Marker.POINTS
        center_marker.action = Marker.ADD
        center_marker.scale.x = 0.12
        center_marker.scale.y = 0.12
        center_marker.color.r = 0.0
        center_marker.color.g = 0.0
        center_marker.color.b = 1.0
        center_marker.color.a = 1.0

        for center_x, center_y, _ in cluster_centers:
            p = Point()
            p.x = center_x
            p.y = center_y
            p.z = 0.10
            center_marker.points.append(p)

        marker_array.markers.append(center_marker)

        blacklist_marker = Marker()
        blacklist_marker.header.frame_id = frame_id
        blacklist_marker.header.stamp = self.get_clock().now().to_msg()
        blacklist_marker.ns = 'frontier_blacklist'
        blacklist_marker.id = 3
        blacklist_marker.type = Marker.POINTS
        blacklist_marker.action = Marker.ADD
        blacklist_marker.scale.x = 0.25
        blacklist_marker.scale.y = 0.25
        blacklist_marker.color.r = 1.0
        blacklist_marker.color.g = 0.5
        blacklist_marker.color.b = 0.0
        blacklist_marker.color.a = 1.0

        for bx, by in self.blacklist:
            p = Point()
            p.x = bx
            p.y = by
            p.z = 0.12
            blacklist_marker.points.append(p)

        marker_array.markers.append(blacklist_marker)

        if best_frontier is not None:
            best_x, best_y, _, _, _ = best_frontier

            best_marker = Marker()
            best_marker.header.frame_id = frame_id
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.ns = 'best_frontier'
            best_marker.id = 4
            best_marker.type = Marker.SPHERE
            best_marker.action = Marker.ADD
            best_marker.pose.position.x = best_x
            best_marker.pose.position.y = best_y
            best_marker.pose.position.z = 0.15
            best_marker.pose.orientation.w = 1.0
            best_marker.scale.x = 0.15
            best_marker.scale.y = 0.15
            best_marker.scale.z = 0.15
            best_marker.color.r = 1.0
            best_marker.color.g = 0.0
            best_marker.color.b = 0.0
            best_marker.color.a = 1.0

            marker_array.markers.append(best_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
