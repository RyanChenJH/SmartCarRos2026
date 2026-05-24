#!/usr/bin/env python3
#coding:UTF-8

import rclpy
from rclpy.node import Node
import serial
import time

from nav_msgs.msg import Odometry

Bytenum_vel = 0               #��ȡ����һ�εĵڼ�λ
Bytenum_dir = 0
last_pose = 0
C = 227.45 / 1000 

#�����־λ����1��-1
#dir=0

# �����ٶ����ݺ���
def DueVelData(inputdata):
    global Bytenum_vel

    for data in inputdata:
        if data == 0x01 and Bytenum_vel == 0: 
            Bytenum_vel = 1
            continue
        if data == 0x03 and Bytenum_vel == 1:
            Bytenum_vel = 2
            continue
        if data == 0x02 and Bytenum_vel == 2:
            Bytenum_vel = 3
            continue
        # ����02λ
        if Bytenum_vel == 3:
            data_high = data
            Bytenum_vel = 4
            continue
        # ����7Aλ
        if Bytenum_vel == 4:
            data_low = data
            Bytenum_vel = 0
            # ��������ս��ٶ�ֵ
            Angle_vel = data_high * 256 + data_low
            
            if Angle_vel >= 32768:
                Angle_vel -= 65536

            # ����
            return float(Angle_vel)

# �����������ݺ��������ձ�������Ȧֵ
def DueDirData(inputdata):
    global Bytenum_dir
    global last_pose

    for data in inputdata:
        if data == 0x01 and Bytenum_dir == 0: 
            Bytenum_dir = 1
            continue
        if data == 0x03 and Bytenum_dir == 1:
            Bytenum_dir = 2
            continue
        if data == 0x02 and Bytenum_dir == 2:
            Bytenum_dir = 3
            continue
        # ����01λ
        if Bytenum_dir == 3:
            data_high = data
            Bytenum_dir = 4
            continue
        # ����42λ
        if Bytenum_dir == 4:
            data_low = data
            Bytenum_dir = 0
            # ��ȡ��������Ȧֵ
            position = data_high * 256 + data_low
            # ��ȥ�ϴε�ֵ
            pose = position - last_pose
            last_pose = position

            # �жϷ���
            if (pose >= 0 and pose < 512) or (pose > -1024 and pose < -512):
                direction = 1
            elif (pose < 0 and pose > -512) or (pose < 1024 and pose > 512):
                direction = -1

            return int(direction)

class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_vel')
        self.pub = self.create_publisher(Odometry, 'encoder', 10)
        
        # ��ȡ����
        self.port = self.declare_parameter('serial_port', '/dev/encoder').value
        self.baud = self.declare_parameter('baud_rate', 57600).value
        self.k = self.declare_parameter('k', 1).value
        
        # ���ö˿ڼ������ʣ�������
        self.ser = serial.Serial(self.port, self.baud)
        self.get_logger().info(f'Serial port {self.port} opened: {self.ser.is_open}')
        
        # ����һ��ѭ����ʱ��������ROS1�е�whileѭ��
        # self.timer = self.create_timer(0.05, self.timer_callback)  # 50Hz
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz

    def timer_callback(self):
        # ���ͻ�ȡ���ٶȵ�����
        send_data = bytes.fromhex('01 03 00 03 00 01 74 0A')
        self.ser.write(send_data)
        datahex = self.ser.read(7)
        angle_v = DueVelData(datahex)

        # ���ͻ�ȡ��������Ȧֵ������
        send_data = bytes.fromhex('01 03 00 00 00 01 84 0A')
        self.ser.write(send_data)
        datahex = self.ser.read(7)
        direction = DueDirData(datahex)
        
        # �������ٶ�
        Vel = 3.57 * angle_v * C / 1024.0 / 0.02 * self.k * 0.25 *1.03
        # Vel = 3.57 * angle_v * C / 1024.0 / 0.02 * self.k * 0.25 *1.05    #encoder_imu 合理

        # ������̼�����
        pub_vel = Odometry()
        pub_vel.header.frame_id = 'odom'
        pub_vel.child_frame_id = 'base_footprint'
        pub_vel.header.stamp = self.get_clock().now().to_msg()
        pub_vel.twist.twist.linear.x = Vel
        self.pub.publish(pub_vel)

def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()