#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


class RaceState(Enum):
    IDLE = 'IDLE'
    ARMED = 'ARMED'
    RACING = 'RACING'
    PARKING = 'PARKING'
    FINISHED = 'FINISHED'
    ERROR = 'ERROR'
    CANCELLED = 'CANCELLED'


class SelectionMode(Enum):
    WAYPOINTS = 'WAYPOINTS'
    PARKING = 'PARKING'


class RaceManager(Node):
    def __init__(self):
        super().__init__('race_manager')

        self.declare_parameter('map_yaml', '')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('cmd_vel_input_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_output_topic', '/car_cmd_vel')
        self.declare_parameter('clicked_point_topic', '/clicked_point')
        self.declare_parameter('preview_topic', '/race/preview_markers')
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('start_x', 0.0)
        self.declare_parameter('start_y', 0.0)
        self.declare_parameter('start_detect_radius', 0.30)
        self.declare_parameter('start_leave_radius', 0.45)
        self.declare_parameter('max_linear_speed', 0.35)
        self.declare_parameter('max_angular_speed', 0.8)
        self.declare_parameter('publish_stop_hz', 20.0)
        self.declare_parameter('cmd_vel_timeout_sec', 0.5)
        self.declare_parameter('waypoints', [])
        self.declare_parameter('parking_names', ['stop_1', 'stop_2'])
        self.declare_parameter('parking_dwell_seconds', [3.0, 5.0])
        self.declare_parameter('parking_point_match_radius', 0.8)
        self.declare_parameter('cancel_nav_during_parking', True)
        self.declare_parameter('print_timer_period', 1.0)
        self.declare_parameter('navigate_action', 'navigate_to_pose')
        self.declare_parameter('action_timeout_sec', 300.0)

        self.odom_topic = self.get_parameter('odom_topic').value
        cmd_vel_input_topic = self.get_parameter('cmd_vel_input_topic').value
        cmd_vel_output_topic = self.get_parameter('cmd_vel_output_topic').value
        clicked_point_topic = self.get_parameter('clicked_point_topic').value
        preview_topic = self.get_parameter('preview_topic').value
        self.goal_frame = self.get_parameter('goal_frame').value
        self.start_x = float(self.get_parameter('start_x').value)
        self.start_y = float(self.get_parameter('start_y').value)
        self.start_detect_radius = float(self.get_parameter('start_detect_radius').value)
        self.start_leave_radius = float(self.get_parameter('start_leave_radius').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.cmd_vel_timeout_sec = float(self.get_parameter('cmd_vel_timeout_sec').value)
        self.parking_point_match_radius = float(self.get_parameter('parking_point_match_radius').value)
        self.cancel_nav_during_parking = bool(self.get_parameter('cancel_nav_during_parking').value)
        self.print_timer_period = float(self.get_parameter('print_timer_period').value)
        navigate_action = self.get_parameter('navigate_action').value
        self.action_timeout_sec = float(self.get_parameter('action_timeout_sec').value)

        self.selected_waypoints = self._parse_waypoints(self.get_parameter('waypoints').value)
        self.selected_parking_points = []
        self.parking_dwell_seconds = [float(v) for v in self.get_parameter('parking_dwell_seconds').value]
        self.parking_names = [str(v) for v in self.get_parameter('parking_names').value]
        self.selection_mode = SelectionMode.WAYPOINTS
        self.points_confirmed = len(self.selected_waypoints) > 0
        self.waypoints = list(self.selected_waypoints)
        self.parking_points = []

        self.state = RaceState.IDLE
        self.current_pose = None
        self.last_odom_pose = None
        self.distance_m = 0.0
        self.current_waypoint_index = 0
        self.current_goal_handle = None
        self.goal_sent_time = None
        self.start_time = None
        self.parking_until = None
        self.parking_index = 0
        self.last_log_time = 0.0
        self.last_cmd_vel_time = None
        self.ignore_cancelled_results = 0
        self.last_passed_waypoint_index = 0

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_output_topic, 10)
        self.preview_pub = self.create_publisher(MarkerArray, preview_topic, 10)
        self.create_subscription(Twist, cmd_vel_input_topic, self.cmd_vel_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 20)
        self.create_subscription(PointStamped, clicked_point_topic, self.clicked_point_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, navigate_action)

        self.create_service(Trigger, '/race/select_waypoints', self.select_waypoints_callback)
        self.create_service(Trigger, '/race/select_parking', self.select_parking_callback)
        self.create_service(Trigger, '/race/clear_points', self.clear_points_callback)
        self.create_service(Trigger, '/race/confirm_points', self.confirm_points_callback)
        self.create_service(Trigger, '/race/arm', self.arm_callback)
        self.create_service(Trigger, '/race/reset', self.reset_callback)
        self.create_service(Trigger, '/race/cancel', self.cancel_callback)

        stop_period = 1.0 / max(float(self.get_parameter('publish_stop_hz').value), 1.0)
        self.create_timer(stop_period, self.control_timer_callback)
        self.create_timer(1.0, self.publish_preview_markers)

        self.get_logger().info('race_manager 已启动：默认接收 RViz /clicked_point 作为导航点；调用 /race/select_parking 后点击停车点；调用 /race/confirm_points 确认后再 /race/arm')

    def _parse_waypoints(self, values):
        values = [float(v) for v in values]
        if len(values) == 0:
            return []
        if len(values) % 3 != 0:
            raise ValueError('waypoints 必须按 [x, y, yaw, ...] 填写，且数量为 3 的倍数')
        return [(values[i], values[i + 1], values[i + 2]) for i in range(0, len(values), 3)]

    def clicked_point_callback(self, msg):
        if self.state not in (RaceState.IDLE, RaceState.CANCELLED, RaceState.FINISHED):
            self.get_logger().warn('比赛已 arm 或正在运行，忽略 RViz 点击点')
            return
        if msg.header.frame_id and msg.header.frame_id != self.goal_frame:
            self.get_logger().warn(f'点击点 frame 是 {msg.header.frame_id}，当前按 {self.goal_frame} 使用')

        if self.selection_mode == SelectionMode.WAYPOINTS:
            yaw = self._estimate_new_waypoint_yaw(msg.point.x, msg.point.y)
            self.selected_waypoints.append((msg.point.x, msg.point.y, yaw))
            self.points_confirmed = False
            self.get_logger().info(f'添加导航点 {len(self.selected_waypoints)}: x={msg.point.x:.2f}, y={msg.point.y:.2f}')
        else:
            if len(self.selected_parking_points) >= len(self.parking_dwell_seconds):
                self.get_logger().warn('停车点数量已达到 parking_dwell_seconds 配置数量，请先 /race/clear_points 或修改参数')
                return
            self.selected_parking_points.append((msg.point.x, msg.point.y))
            self.points_confirmed = False
            self.get_logger().info(f'添加停车点 {len(self.selected_parking_points)}: x={msg.point.x:.2f}, y={msg.point.y:.2f}')
        self.publish_preview_markers()

    def select_waypoints_callback(self, request, response):
        del request
        self.selection_mode = SelectionMode.WAYPOINTS
        response.success = True
        response.message = '已切换为导航点采集模式，在 RViz 使用 Publish Point 依次点击导航点'
        self.get_logger().info(response.message)
        return response

    def select_parking_callback(self, request, response):
        del request
        self.selection_mode = SelectionMode.PARKING
        response.success = True
        response.message = '已切换为停车点采集模式，在 RViz 使用 Publish Point 点击两个停车位置'
        self.get_logger().info(response.message)
        return response

    def clear_points_callback(self, request, response):
        del request
        if self.state not in (RaceState.IDLE, RaceState.CANCELLED, RaceState.FINISHED):
            response.success = False
            response.message = '比赛已 arm 或正在运行，不能清空点位'
            return response
        self.selected_waypoints.clear()
        self.selected_parking_points.clear()
        self.points_confirmed = False
        self.waypoints = []
        self.parking_points = []
        self._publish_delete_all_markers()
        response.success = True
        response.message = '已清空导航点和停车点'
        self.get_logger().info(response.message)
        return response

    def confirm_points_callback(self, request, response):
        del request
        if len(self.selected_waypoints) == 0:
            response.success = False
            response.message = '还没有导航点，请先在 RViz 中点击导航点'
            return response
        if len(self.selected_parking_points) != len(self.parking_dwell_seconds):
            response.success = False
            response.message = f'停车点数量应为 {len(self.parking_dwell_seconds)}，当前为 {len(self.selected_parking_points)}'
            return response
        self.waypoints = self._waypoints_with_yaw(self.selected_waypoints)
        self.parking_points = self._make_parking_points_from_selected()
        self.points_confirmed = True
        self.publish_preview_markers()
        response.success = True
        response.message = f'点位已确认：导航点 {len(self.waypoints)} 个，停车点 {len(self.parking_points)} 个，可以调用 /race/arm'
        self.get_logger().info(response.message)
        return response

    def odom_callback(self, msg):
        pose = msg.pose.pose.position
        current = (pose.x, pose.y)
        self.current_pose = current

        if self.state == RaceState.RACING and self.last_odom_pose is not None:
            delta = math.hypot(current[0] - self.last_odom_pose[0], current[1] - self.last_odom_pose[1])
            if 0.001 <= delta <= 0.5:
                self.distance_m += delta

        if self.state == RaceState.RACING:
            self.last_odom_pose = current

    def cmd_vel_callback(self, msg):
        if self.state != RaceState.RACING:
            return
        limited = Twist()
        limited.linear.x = self._clamp(msg.linear.x, -self.max_linear_speed, self.max_linear_speed)
        limited.angular.z = self._clamp(msg.angular.z, -self.max_angular_speed, self.max_angular_speed)
        self.last_cmd_vel_time = self._now_sec()
        self.cmd_pub.publish(limited)

    def arm_callback(self, request, response):
        del request
        if not self.points_confirmed:
            response.success = False
            response.message = '点位还未确认，请先调用 /race/confirm_points'
            return response
        if self.current_pose is None:
            response.success = False
            response.message = '还没有收到里程计，无法 arm'
            return response
        distance_to_start = self._distance_to_start()
        if distance_to_start > self.start_detect_radius:
            response.success = False
            response.message = f'小车不在起点附近，当前距离起点 {distance_to_start:.2f} m'
            return response
        self._reset_runtime_state()
        self.state = RaceState.ARMED
        self._publish_stop()
        response.success = True
        response.message = '已 arm，等待小车离开起点后开始计时'
        self.get_logger().info(response.message)
        return response

    def reset_callback(self, request, response):
        del request
        self._cancel_current_goal()
        self._reset_runtime_state()
        self.state = RaceState.IDLE
        self._publish_stop()
        response.success = True
        response.message = '比赛状态已重置，已保留已确认点位'
        self.get_logger().info(response.message)
        return response

    def cancel_callback(self, request, response):
        del request
        self._cancel_current_goal()
        self.state = RaceState.CANCELLED
        self._publish_stop()
        response.success = True
        response.message = '比赛已取消，车辆停止'
        self.get_logger().warn(response.message)
        return response

    def control_timer_callback(self):
        now = self._now_sec()

        if self.state == RaceState.ARMED:
            self._publish_stop()
            if self._distance_to_start() > self.start_leave_radius:
                self._start_race()
            return

        if self.state == RaceState.RACING:
            self._check_parking_points()
            self._check_action_timeout(now)
            self._check_cmd_vel_timeout(now)
            self._log_status(now)
            return

        if self.state == RaceState.PARKING:
            self._publish_stop()
            if self.parking_until is not None and now >= self.parking_until:
                self.get_logger().info('停车结束，恢复导航')
                self.state = RaceState.RACING
                self.last_odom_pose = self.current_pose
                self._send_current_goal()
            return

        if self.state in (RaceState.IDLE, RaceState.FINISHED, RaceState.ERROR, RaceState.CANCELLED):
            self._publish_stop()

    def _start_race(self):
        self.state = RaceState.RACING
        self.start_time = self._now_sec()
        self.distance_m = 0.0
        self.last_odom_pose = self.current_pose
        self.current_waypoint_index = 0
        self.parking_index = 0
        self.last_passed_waypoint_index = 0
        for point in self.parking_points:
            point['done'] = False
        self.get_logger().info('小车已离开起点，比赛开始计时')
        self._send_current_goal()

    def _send_current_goal(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self._finish_race()
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.state = RaceState.ERROR
            self.get_logger().error('Nav2 navigate_to_pose action 服务不可用')
            return

        x, y, yaw = self.waypoints[self.current_waypoint_index]
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
        self.goal_sent_time = self._now_sec()
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.get_logger().info(f'发送导航点 {self.current_waypoint_index + 1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.state = RaceState.ERROR
            self.get_logger().error('导航目标被 Nav2 拒绝')
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        status = future.result().status
        if self.state == RaceState.PARKING:
            return
        if self.ignore_cancelled_results > 0 and status in (GoalStatus.STATUS_CANCELING, GoalStatus.STATUS_CANCELED):
            self.ignore_cancelled_results -= 1
            return
        if self.state != RaceState.RACING:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'导航点 {self.current_waypoint_index + 1} 已到达')
            self.last_passed_waypoint_index = self.current_waypoint_index
            self.current_waypoint_index += 1
            self.current_goal_handle = None
            self._send_current_goal()
            return
        self.state = RaceState.ERROR
        self.current_goal_handle = None
        self.get_logger().error(f'导航失败，状态码: {status}')

    def _check_parking_points(self):
        if self.parking_index >= len(self.parking_points):
            return
        point = self.parking_points[self.parking_index]
        if point['done']:
            return
        if self._distance_to_point(point['x'], point['y']) > self.parking_point_match_radius:
            return
        point['done'] = True
        self.parking_index += 1
        self.state = RaceState.PARKING
        self.parking_until = self._now_sec() + point['dwell']
        if self.cancel_nav_during_parking:
            self._cancel_current_goal()
        self._publish_stop()
        self.get_logger().info(f"到达停车点 {point['name']}，停车 {point['dwell']:.1f} s")

    def _check_action_timeout(self, now):
        if self.goal_sent_time is None:
            return
        if now - self.goal_sent_time <= self.action_timeout_sec:
            return
        self.state = RaceState.ERROR
        self._cancel_current_goal()
        self.get_logger().error('当前导航点超时，比赛进入 ERROR')

    def _check_cmd_vel_timeout(self, now):
        if self.last_cmd_vel_time is None:
            return
        if now - self.last_cmd_vel_time > self.cmd_vel_timeout_sec:
            self._publish_stop()

    def _finish_race(self):
        self.state = RaceState.FINISHED
        self._publish_stop()
        total_time = 0.0 if self.start_time is None else self._now_sec() - self.start_time
        self.get_logger().info(f'比赛完成，总用时 {total_time:.2f} s，总里程 {self.distance_m:.2f} m')

    def _cancel_current_goal(self):
        if self.current_goal_handle is not None:
            self.ignore_cancelled_results += 1
            self.current_goal_handle.cancel_goal_async()
            self.current_goal_handle = None

    def _reset_runtime_state(self):
        self.start_time = None
        self.distance_m = 0.0
        self.current_waypoint_index = 0
        self.current_goal_handle = None
        self.goal_sent_time = None
        self.parking_until = None
        self.parking_index = 0
        self.last_odom_pose = None
        self.last_cmd_vel_time = None
        self.last_passed_waypoint_index = 0
        for point in self.parking_points:
            point['done'] = False

    def _make_parking_points_from_selected(self):
        points = []
        for index, point in enumerate(self.selected_parking_points):
            name = self.parking_names[index] if index < len(self.parking_names) else f'stop_{index + 1}'
            dwell = self.parking_dwell_seconds[index]
            points.append({'name': name, 'x': point[0], 'y': point[1], 'dwell': dwell, 'done': False})
        return points

    def _estimate_new_waypoint_yaw(self, x, y):
        if not self.selected_waypoints:
            return 0.0
        last_x, last_y, _ = self.selected_waypoints[-1]
        return math.atan2(y - last_y, x - last_x)

    def _waypoints_with_yaw(self, waypoints):
        result = []
        for index, point in enumerate(waypoints):
            x, y, yaw = point
            if index + 1 < len(waypoints):
                next_x, next_y, _ = waypoints[index + 1]
                yaw = math.atan2(next_y - y, next_x - x)
            elif index > 0:
                prev_x, prev_y, _ = waypoints[index - 1]
                yaw = math.atan2(y - prev_y, x - prev_x)
            result.append((x, y, yaw))
        return result

    def publish_preview_markers(self):
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        for index, (x, y, yaw) in enumerate(self._waypoints_with_yaw(self.selected_waypoints)):
            marker = Marker()
            marker.header.frame_id = self.goal_frame
            marker.header.stamp = now
            marker.ns = 'race_waypoints'
            marker.id = index
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.08
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x = 0.45
            marker.scale.y = 0.08
            marker.scale.z = 0.08
            marker.color.a = 1.0
            marker.color.g = 1.0
            markers.markers.append(marker)

            text = self._make_text_marker('race_waypoint_text', 1000 + index, x, y, f'P{index + 1}', 0.0, 1.0, 0.0)
            markers.markers.append(text)

        for index, (x, y) in enumerate(self.selected_parking_points):
            marker = Marker()
            marker.header.frame_id = self.goal_frame
            marker.header.stamp = now
            marker.ns = 'race_parking'
            marker.id = 2000 + index
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.pose.orientation.w = 1.0
            marker.scale.x = self.parking_point_match_radius * 2.0
            marker.scale.y = self.parking_point_match_radius * 2.0
            marker.scale.z = 0.10
            marker.color.a = 0.45
            marker.color.r = 1.0
            marker.color.g = 0.8
            markers.markers.append(marker)

            name = self.parking_names[index] if index < len(self.parking_names) else f'stop_{index + 1}'
            dwell = self.parking_dwell_seconds[index] if index < len(self.parking_dwell_seconds) else 0.0
            text = self._make_text_marker('race_parking_text', 3000 + index, x, y, f'{name}\n{dwell:.1f}s', 1.0, 0.7, 0.0)
            markers.markers.append(text)

        self.preview_pub.publish(markers)

    def _make_text_marker(self, namespace, marker_id, x, y, text, r, g, b):
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.35
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.35
        marker.color.a = 1.0
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.text = text
        return marker

    def _publish_delete_all_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        markers = MarkerArray()
        markers.markers.append(marker)
        self.preview_pub.publish(markers)

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    def _distance_to_start(self):
        if self.current_pose is None:
            return float('inf')
        return math.hypot(self.current_pose[0] - self.start_x, self.current_pose[1] - self.start_y)

    def _distance_to_point(self, x, y):
        if self.current_pose is None:
            return float('inf')
        return math.hypot(self.current_pose[0] - x, self.current_pose[1] - y)

    def _log_status(self, now):
        if now - self.last_log_time < self.print_timer_period:
            return
        self.last_log_time = now
        elapsed = 0.0 if self.start_time is None else now - self.start_time
        self.get_logger().info(f'状态={self.state.value} 用时={elapsed:.1f}s 里程={self.distance_m:.2f}m 路点={self.current_waypoint_index + 1}/{len(self.waypoints)}')

    def _now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _clamp(value, low, high):
        return max(low, min(high, value))


def main(args=None):
    rclpy.init(args=args)
    node = RaceManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
