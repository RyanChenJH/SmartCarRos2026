#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
import time
import numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile
import cv_bridge
from geometry_msgs.msg import Twist
last_erro=0
tmp_cv = 0
def nothing(s):
    pass
col_black = (0,0,0,180,255,46)# black
col_red = (0,100,80,10,255,255)# red
col_blue = (100,43,46,124,255,255)# blue
col_green= (35,43,46,77,255,255)# green
col_yellow = (26,43,46,34,255,255)# yellow

Switch = '0:Red\n1:Green\n2:Blue\n3:Yellow\n4:Black'

class Follower(Node):
    def __init__(self):
        super().__init__('follower')
        self.bridge = cv_bridge.CvBridge()
        qos = QoSProfile(depth=10)
        self.mat = None
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',#订阅摄像头话题
            self.image_callback,
            qos)
        self.cmd_vel_pub = self.create_publisher(Twist, 'car_cmd_vel', qos)
        self.twist = Twist()
        self.tmp = 0

    def image_callback(self, msg):
        global last_erro
        global tmp_cv
        if self.tmp == 0:
            cv2.namedWindow('Adjust_hsv', cv2.WINDOW_NORMAL)
            cv2.createTrackbar(Switch, 'Adjust_hsv', 0, 4, nothing)
            self.tmp = 1

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv_blur = cv2.GaussianBlur(hsv, (5, 5), 0)

        kernel = numpy.ones((5, 5), numpy.uint8)
        hsv_erode = cv2.erode(hsv_blur, kernel, iterations=1)
        hsv_dilate = cv2.dilate(hsv_erode, kernel, iterations=1)

        # 获取滑块选择的颜色
        m = cv2.getTrackbarPos(Switch, 'Adjust_hsv')
        if m == 0:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = col_red
        elif m == 1:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = col_green
        elif m == 2:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = col_blue
        elif m == 3:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = col_yellow
        elif m == 4:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = col_black
        else:
            lowerbH, lowerbS, lowerbV, upperbH, upperbS, upperbV = 0, 0, 0, 255, 255, 255

        # 创建掩膜
        mask = cv2.inRange(hsv_dilate, (lowerbH, lowerbS, lowerbV), (upperbH, upperbS, upperbV))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        h, w, d = image.shape

        # 分割左右区域
        left_mask = mask.copy()
        right_mask = mask.copy()
        left_mask[:, w // 2:] = 0  # 左侧区域
        right_mask[:, :w // 2] = 0  # 右侧区域

        # 计算左侧线的重心
        M_left = cv2.moments(left_mask)
        cx_left = int(M_left['m10'] / M_left['m00']) if M_left['m00'] > 0 else None

        # 计算右侧线的重心
        M_right = cv2.moments(right_mask)
        cx_right = int(M_right['m10'] / M_right['m00']) if M_right['m00'] > 0 else None

        # 计算中点
        if cx_left is not None and cx_right is not None:
            cx_mid = (cx_left + cx_right) // 2
            erro = cx_mid - w // 2
            Kp = 0.02  # 正常比例系数
        elif cx_left is not None:
            erro = cx_left - w // 2
            Kp = 0.12  # 单边时增大比例系数
        elif cx_right is not None:
            erro = cx_right - w // 2
            Kp = 0.12  # 单边时增大比例系数
        else:
            erro = 0
            Kp = 0.12
        max_angular = 1.0
        # 计算角速度
        angular_z = -Kp * erro

        # 线速度策略：角度越大，速度越小；小角度保持高速度
        if abs(erro) < 10:
            linear_x = 0.30
        elif abs(erro) < 50:
            linear_x = 0.25
        elif abs(erro) < 100:
            linear_x = 0.20
        else:
            linear_x = 0.0

        # 设置并发布速度
        self.twist.linear.x = linear_x
        self.twist.angular.z = angular_z
        self.cmd_vel_pub.publish(self.twist)
        # 打印调试信息
        print(f'erro={erro}, angular_z={angular_z:.2f}, linear_x={linear_x:.2f}')

        # 检测边缘
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)

        # 显示调试窗口
        cv2.imshow("Adjust_hsv", mask)
        cv2.waitKey(3)

def main(args=None):
    rclpy.init(args=args)
    print('start patrolling')
    follower = Follower()
    while rclpy.ok():
        rclpy.spin_once(follower)
        time.sleep(0.1)

    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
