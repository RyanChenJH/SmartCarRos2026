#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Twist
import csv

class NavThroughPosesClient(Node):
    def __init__(self):
        # 初始化节点
        super().__init__('nav_through_poses_client')
        # 创建 NavigateThroughPoses 动作客户端
        self._action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        # 订阅 cmd_vel 话题
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # 创建 car_cmd_vel 话题的发布者
        self.car_cmd_vel_pub = self.create_publisher(Twist, '/car_cmd_vel', 10)
        # 初始化剩余距离、暂停标志位、循环计数和完成标志
        self.distance_remaining = None
        self.is_paused = False  # 标志位，用于控制是否暂停导航
        self.cycle_number = 0
        self.fin_flag = False

    def send_goal(self, poses):
        # 创建目标消息
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        # 等待服务器连接
        self._action_client.wait_for_server(timeout_sec=10.0)
        # 发送目标并注册回调
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # 处理目标响应
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('目标被拒绝 :(')
            return

        self.get_logger().info('目标已接受 :)')

        # 获取结果并注册回调
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # 处理结果
        result = future.result().result
        self.get_logger().info('导航结果: {0}'.format(result))
        # 如果导航完成，设置完成标志
        if result.navigation_time.sec > 0:
            self.fin_flag = True
            # 重新发送目标以实现循环导航
            self.send_goal(self.poses)

    def feedback_callback(self, feedback_msg):
        # 处理反馈信息
        feedback = feedback_msg.feedback
        if feedback.distance_remaining > 0.0:
            self.get_logger().info(f'到目标的距离: {feedback.distance_remaining:.2f} 米')
            self.distance_remaining = feedback.distance_remaining
            # 如果距离目标点的距离小于 0.5 米且未暂停，停止机器人
            if feedback.distance_remaining < 0.5 and not self.is_paused:
                self.get_logger().info('到达目标点，停止机器人。')
                self.is_paused = True  # 设置标志位为暂停状态
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.car_cmd_vel_pub.publish(twist)

    def cmd_vel_callback(self, msg):
        # 如果剩余距离大于 0.5 米且未暂停，正常转发速度命令
        if self.distance_remaining > 0.5 and not self.is_paused:
            self.car_cmd_vel_pub.publish(msg)

def read_waypoints_from_csv(filename):
    # 从 CSV 文件读取导航点
    waypoints = []
    with open(filename, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if len(row) == 4:  # 确保每行有 x, y, z, w
                waypoints.append({
                    "x": float(row[0]),
                    "y": float(row[1]),
                    "z": float(row[2]),
                    "w": float(row[3])
                })
    return waypoints

def main(args=None):
    rclpy.init(args=args)

    client = NavThroughPosesClient()

    # 从 CSV 文件读取导航点
    waypoints = read_waypoints_from_csv('/home/davinci-mini/CHEN2/src/racecar/scripts/ai_test.csv')

    # 将导航点转换为 PoseStamped 消息
    poses = []
    for point in waypoints:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp.sec = 0
        pose.pose.position.x = point["x"]
        pose.pose.position.y = point["y"]
        pose.pose.position.z = point["z"]
        pose.pose.orientation.w = point["w"]
        poses.append(pose)
    client.poses = poses  # 保存 poses 到 client 中

    client.send_goal(poses)

    rclpy.spin(client)

if __name__ == '__main__':
    main()