#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import math

class NavigationGoalPublisher(Node):
    def __init__(self):
        super().__init__('navigation_goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.plan_subscriber = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.odom_subscriber = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)
        
        self.goal_received = False
        self.goal_reached = False
        self.current_goal = self.create_goal_pose(-2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        self.timer = self.create_timer(1.0, self.publish_goal)
        
    def create_goal_pose(self, x, y, z, ox, oy, oz, ow):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.x = ox
        goal_msg.pose.orientation.y = oy
        goal_msg.pose.orientation.z = oz
        goal_msg.pose.orientation.w = ow
        return goal_msg

    def publish_goal(self):
        if not self.goal_received:
            self.current_goal.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(self.current_goal)
            self.get_logger().info('Published goal pose')
        elif self.goal_reached:
            self.goal_received = False
            self.goal_reached = False
            self.current_goal = self.create_goal_pose(-2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0)
            self.timer.reset()

    def plan_callback(self, msg):
        if msg.poses:
            self.goal_received = True
            self.get_logger().info('Received valid plan')
            self.timer.cancel()

    def odom_callback(self, msg):
        if self.goal_received:
            current_position = msg.pose.pose.position
            goal_position = self.current_goal.pose.position
            distance = math.sqrt(
                (current_position.x - goal_position.x) ** 2 +
                (current_position.y - goal_position.y) ** 2
            )
            if distance < 0.5:
                self.goal_reached = True
                self.get_logger().info('Reached goal pose')

def main(args=None):
    rclpy.init(args=args)
    navigation_goal_publisher = NavigationGoalPublisher()
    rclpy.spin(navigation_goal_publisher)
    navigation_goal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()