import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.clock import Clock
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import numpy as np

PI = 3.14159

class EncoderImuMixNode(Node):
    def __init__(self):
        super().__init__('encoder_imu_mix')
        self.current_time = Clock().now()
        self.last_time = Clock().now()

        self.x = 0.0
        self.y = 0.0
        self.s = 0.0
        self.th = 0.0
        self.vth = 0.0
        self.th_init = 0.0
        self.theta_first = 0.0
        self.flag = True

        self.encoder_imu_pub = self.create_publisher(Odometry, '/encoder_imu_odom', 10)
        
        self.imu_sub = self.create_subscription(Imu, '/IMU_data', self.imu_callback, 10)
        self.encoder_sub = self.create_subscription(Odometry, '/encoder', self.encoder_callback, 10)
        
        self.imu_data = None
        self.encoder_data = None

    def get_yaw_from_pose(self, imu_data):
        orientation_q = imu_data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def get_roll_from_pose(self, imu_data):
        orientation_q = imu_data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, _, _ = euler_from_quaternion(orientation_list)
        return roll

    def get_pitch_from_pose(self, imu_data):
        orientation_q = imu_data.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, pitch, _ = euler_from_quaternion(orientation_list)
        return pitch

    def imu_callback(self, imu_data):
        self.imu_data = imu_data
        if self.encoder_data is not None:
            self.callback()

    def encoder_callback(self, encoder_data):
        self.encoder_data = encoder_data
        if self.imu_data is not None:
            self.callback()

    def callback(self):
        if self.flag:
            self.theta_first = self.get_yaw_from_pose(self.imu_data)
            self.flag = False

        if not self.flag:
            v = self.encoder_data.twist.twist.linear.x
            theta = self.get_yaw_from_pose(self.imu_data)
            roll = self.get_roll_from_pose(self.imu_data)
            pitch = self.get_pitch_from_pose(self.imu_data)
            theta -= self.theta_first

            self.th_init = self.theta_first * 180 / PI
            self.th = theta * 180 / PI

            vx = v * math.cos(self.th / 180 * PI)
            vy = v * math.sin(self.th / 180 * PI)
            self.current_time = Clock().now()
            dt = (self.current_time - self.last_time).nanoseconds / 1e9
            delta_x = vx * dt
            delta_y = vy * dt

            self.x += delta_x
            self.y += delta_y
            self.s = math.sqrt(self.x**2 + self.y**2)

            self.get_logger().info(f'v    :{v:.4f}')
            self.get_logger().info(f'theta:{self.th:.4f}')
            self.get_logger().info(f'length:{self.s * 100:.1f}')

            odom = Odometry()
            odom.header.stamp = self.current_time.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_footprint'

            quat = quaternion_from_euler(roll, pitch, theta)
            odom_quat = Quaternion()
            odom_quat.x = quat[0]
            odom_quat.y = quat[1]
            odom_quat.z = quat[2]
            odom_quat.w = quat[3]

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation = odom_quat

            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = 0

            self.encoder_imu_pub.publish(odom)

            self.last_time = self.current_time


def main(args=None):
    rclpy.init(args=args)
    node = EncoderImuMixNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()