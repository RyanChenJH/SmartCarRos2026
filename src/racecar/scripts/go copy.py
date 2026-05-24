#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64
import csv
import numpy as np
import math

class MultiGoals(Node):
    def __init__(self, goalListX, goalListY, goalListZ, goalListW, retry, map_frame):
        super().__init__('multi_goals')
        self.retry = retry
        self.pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_ekf_sub = self.create_subscription(Odometry, '/odometry/filtered', self.getPose_ekf, 10)
        self.plan_sub = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.pub_final = self.create_publisher(Float64, '/arrfinal', 1)
        
        self.goalListX = goalListX
        self.goalListY = goalListY
        self.goalListZ = goalListZ
        self.goalListW = goalListW
        self.kx = 0
        self.ky = 0
        self.gx = 0
        self.gy = 0
        self.flag = 1
        self.MIN_DISTANCE = 1.5  # min distance of the judge between the goal and odometry pose
        self.LONG = len(self.goalListX)
        self.goalId = 0
        self.count = 0
        self.start_time = 0
        self.pubfinal = False
        self.goalMsg = PoseStamped()
        self.goalMsg.header.frame_id = map_frame
        self.plan_received = False

        # Timer to continuously publish goals
        self.timer = self.create_timer(1.0, self.publish_goal)

    def publish_goal(self):
        if self.goalId >= len(self.goalListX):
            self.get_logger().info("All goals have been published.")
            self.timer.cancel()
            return

        if not self.plan_received:
            self.goalMsg.header.stamp = self.get_clock().now().to_msg()
            self.goalMsg.pose.position.x = self.goalListX[self.goalId]
            self.goalMsg.pose.position.y = self.goalListY[self.goalId]
            self.goalMsg.pose.orientation.z = self.goalListZ[self.goalId]
            self.goalMsg.pose.orientation.w = self.goalListW[self.goalId]
            self.pub.publish(self.goalMsg)
            self.start_time = self.get_clock().now().to_msg().sec
            self.get_logger().info(f"Published goal! Goal ID: {self.goalId}")

    def plan_callback(self, msg):
        if msg.poses:
            self.plan_received = True
            self.get_logger().info("Received valid plan.")
            self.timer.cancel()

    def statusCB(self):
        if self.goalId >= len(self.goalListX):
            return

        self.gx = self.goalListX[self.goalId]
        self.gy = self.goalListY[self.goalId]

        self.dist = self.distance(self.kx, self.ky, self.gx, self.gy)

        if self.dist < self.MIN_DISTANCE and self.flag == 1:
            finish_time = self.get_clock().now().to_msg().sec
            interval = finish_time - self.start_time
            self.get_logger().info(f"Reached goal in {interval} seconds")

            self.goalId += 1
            self.plan_received = False
            if self.goalId < len(self.goalListX):
                self.timer.reset()
                self.publish_goal()

    def getPose_ekf(self, data):
        self.kx = data.pose.pose.position.x
        self.ky = data.pose.pose.position.y
        self.statusCB()

    def distance(self, kx, ky, gx, gy):
        return math.sqrt((kx - gx) ** 2 + (ky - gy) ** 2)

def main(args=None):
    rclpy.init(args=args)

    goalListX = []
    goalListY = []
    goalListZ = []
    goalListW = []

    map_frame = 'map'

    with open('test.csv', 'r') as f:
        reader = csv.reader(f)
        goalList = [list(map(float, cols)) for cols in reader]

    goalList = np.array(goalList)
    print("read suc!!")
    goalListX = goalList[:, 0]
    goalListY = goalList[:, 1]
    goalListZ = goalList[:, 2]
    goalListW = goalList[:, 3]

    if len(goalListX) == len(goalListY) and len(goalListY) >= 1:
        print("Multi Goals Executing...")
        mg = MultiGoals(goalListX, goalListY, goalListZ, goalListW, 1, map_frame)
        rclpy.spin(mg)
    else:
        print("Lengths of goal lists are not the same")

    mg.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("shutting down")