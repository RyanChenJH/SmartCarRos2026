#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os
import math
import sys, select, termios, tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send_points(data):
    global already_send_flag
    already_send_flag = 1

def getPose(data):
    global kx
    global ky
    kx = data.pose.pose.position.x
    ky = data.pose.pose.position.y

def cv_show(name,img):
    cv2.imshow(name,img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

class RacecarNode(Node):
    def __init__(self):
        super().__init__('run_car')
        self.publisher_ = self.create_publisher(Twist, '/car_cmd_vel', 10)
        self.twist = Twist()
        self.cap = cv2.VideoCapture(0)
        time.sleep(2)
        self.hight = 750
        self.weight = 349
        self.First_Into_Right = 0
        self.First_Into_Left = 0
        self.straight_flag = 0
        self.circle_k = 26
        self.straight_k = 23
        self.flag_4 = 0
        self.times = 0
        self.settings = termios.tcgetattr(sys.stdin)
        self.start_time = time.time()

    def shibie(self, image):
        now_time = time.time() - self.start_time
        if 15 <= now_time < 19:
            self.twist.linear.x = 0.30
            self.twist.angular.z = 0.5  
            self.publisher_.publish(self.twist)
            print(f"正在左转，当前时间: {now_time:.2f}s")
            print(f"程序运行时间: {now_time:.2f} 秒")  # 实时打印
            return  # 跳过后续识别

        # 裁剪图像，去除地面区域
        h, w = image.shape[:2]
        # 只保留上2/3区域
        roi_img = image[0:int(h*2/3), :]
        start_img = roi_img.copy()

        # 后续处理都用start_img
        b, g, r = cv2.split(start_img)
        Hsv_img = cv2.merge([b, r, g])
        Hsv_img = cv2.cvtColor(Hsv_img, cv2.COLOR_BGR2HSV)
        low_blue = np.array([90, 110, 28])
        high_blue = np.array([170, 255, 255])
        low_red= np.array([50, 100, 60])
        high_red = np.array([70, 255, 255])
        Hsv_img2 = cv2.inRange(Hsv_img, low_red,high_red)
        Hsv_img1 = cv2.inRange(Hsv_img, low_blue, high_blue)

        contours, hierarchy = cv2.findContours(Hsv_img1, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE) 
        contours_red, hierarchy_red = cv2.findContours(Hsv_img2,cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE) 
        blue_cone_x = 0
        blue_cone_y = 0
        blue_count = 0
        red_cone_y = 0
        red_cone_x = 0
        red_cone_i=0
        red_count = 0
        right_blue_point_row =0
        right_blue_point_rol = 0
        left_red_point_row = 0
        left_red_point_rol = 0
        
        if self.flag_4 == 1:
            for i in contours:                     
                x, y, w, h = cv2.boundingRect(i)
                area =  0-cv2.contourArea(i,True)
                if area < 325:
                    continue
                blue_count = blue_count+1
                if y+h > blue_cone_y:
                    blue_cone_y = y+h
                    blue_cone_i = i
                    blue_cone_x = x+w
                
        else :
            for i in contours:                    
                x, y, w, h = cv2.boundingRect(i)
                area =  0-cv2.contourArea(i,True)
                if area < 325:
                    continue
                blue_count = blue_count+1
                if y+h+w+x > blue_cone_y+blue_cone_x:
                    blue_cone_y = y+h
                    blue_cone_i = i
                    blue_cone_x = x+w

        for i in contours_red:
            x, y, w, h = cv2.boundingRect(i)
            area = 0 - cv2.contourArea(i, True)
            if area < 325:#1300
                continue
            red_count = red_count+1
            if y+h > red_cone_y:
                red_cone_y = y+h
                red_cone_x = x+w
                red_cone_i = i
        if red_cone_x > blue_cone_x:
            blue_count = 0
        if red_count!=0:
            x, y, w, h = cv2.boundingRect(red_cone_i)
            brcnt = np.array([[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]])
            # 画红色矩形框
            cv2.rectangle(start_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.circle(start_img, (int((2*x+w)/2), y+h), 2, (255, 255, 255), 2)
            left_red_point_row = int((2*x+w)/2)
            left_red_point_rol = y+h
            #显示坐标
            #cv2.putText(image, f'({left_red_point_row}, {left_red_point_rol})', (left_red_point_row, left_red_point_rol - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        if blue_count!=0 :
            x, y, w, h = cv2.boundingRect(blue_cone_i)
            brcnt = np.array([[[x, y]], [[x + w, y]], [[x + w, y + h]], [[x, y + h]]])
            # 画蓝色矩形框
            cv2.rectangle(start_img, (x, y), (x + w, y + h), (255, 0, 0), 2)
            right_blue_point_row =int((2*x+w)/2)
            right_blue_point_rol = y+h
            #显示坐标
            #cv2.putText(image, f'({right_blue_point_row}, {right_blue_point_rol})', (right_blue_point_row, right_blue_point_rol - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
        flag = 0
        angle1 = 0
        middle = 90
        limit = 5
        k = 90

        Left_Null_Flag = 0
        Right_Null_Flag = 0
        if blue_count == 0 :
            right_blue_point_row = 320
            right_blue_point_rol = 120
            Right_Null_Flag  = 1
            
        if left_red_point_row > 120 and self.First_Into_Left  == 0:
            self.First_Into_Left = 1
            if self.First_Into_Left == 1:
                Right_Null_Flag  = 1
                
        if red_count == 0 :
            left_red_point_row = 0
            left_red_point_rol = 120
            Left_Null_Flag = 1
            
        if right_blue_point_row >120 and self.First_Into_Right == 0:
            self.First_Into_Right = 2
            if  self.First_Into_Right ==2:
                Left_Null_Flag = 1
                    
        if red_count != 0:
            self.First_Into_Right = 0
        if blue_count != 0:
            self.First_Into_Left  = 0
        #计算红蓝色的中点（横row,纵rol）
        row = int((left_red_point_row + right_blue_point_row)/2)
        rol = int((right_blue_point_rol + left_red_point_rol)/2)
        angle = (160- row)*2 -((left_red_point_rol - right_blue_point_rol) * 1)
        angle1 = 0

        if Left_Null_Flag ==0 and Right_Null_Flag ==0:
            if angle>0 and angle <limit :
                flag = 1
            if angle >-limit and angle < 0:
                flag = 1
        if flag == 0:
            if angle > limit:
                angle = angle - limit
            else :
                angle = angle + limit
            if angle > 100:
                angle = 100
            if angle < -100:
                angle = -100
            if self.straight_flag == 0:
                angle1 = (angle/100)*self.circle_k
            else :
                angle1 = (angle/100)*self.straight_k
        angle = str(angle1)
        #cv2.putText(start_img, angle, (160, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) ,
                #          thickness=1, lineType=cv2.LINE_AA)
        if flag ==0:
            k =  middle + 1.0*angle1#2.25
        else:
            k = middle
        self.twist.linear.x = 0.25
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = float(k)  # k如果是int，需转float
        self.publisher_.publish(self.twist)
        cv2.imshow('Image1',start_img)

        # 计算角度，angle1为归一化到[-1, 1]的转向量
        max_angle = 100  # 你可以根据实际需要调整
        angle = (160 - row) * 2 - ((left_red_point_rol - right_blue_point_rol) * 1)
        angle1 = 0.0

        if Left_Null_Flag == 0 and Right_Null_Flag == 0:
            if angle > 0 and angle < limit:
                flag = 1
            if angle > -limit and angle < 0:
                flag = 1
        if flag == 0:
            if angle > limit:
                angle = angle - limit
            else:
                angle = angle + limit
            # 限制angle在[-max_angle, max_angle]
            angle = max(-max_angle, min(angle, max_angle))
            # 归一化到[-1, 1]
            angle1 = angle / max_angle
        else:
            angle1 = 0.0

        angle1 = angle / max_angle

        # ...计算angle1后，加入平滑处理...
        self.last_angle1 = getattr(self, 'last_angle1', 0.0)
        alpha = 0.7  # 越大越平滑
        angle1 = alpha * self.last_angle1 + (1 - alpha) * angle1
        self.last_angle1 = angle1

        # 速度和角度赋值
        self.twist.linear.x = 0.25
        self.twist.angular.z = float(angle1)
        self.publisher_.publish(self.twist)

        self.last_angle1 = getattr(self, 'last_angle1', 0.0)
        alpha = 0.8
        angle1 = alpha * self.last_angle1 + (1 - alpha) * angle1
        self.last_angle1 = angle1
        self.twist.linear.x = 0.26
        self.twist.angular.z = float(angle1)
        self.publisher_.publish(self.twist)
        cv2.imshow('Image1', start_img)
        print(f"红锥桶数量(red_count): {red_count}")
        print(f"蓝锥桶数量(blue_count): {blue_count}")
        print(f"发布角度(angular.z): {self.twist.angular.z}")
        print(f"发布速度(linear.x): {self.twist.linear.x}")
        now_time = time.time() - self.start_time
        print(f"程序运行时间: {now_time:.2f} 秒")

    def Camera_vision(self):
        j = 0
        while True:
            key = getKey(self.settings)
            self.times += 1
            ret, frame = self.cap.read()
            if not ret:
                break
            frame1 = cv2.resize(frame, (320, 240), interpolation=cv2.INTER_CUBIC)
            self.shibie(frame1)

            if key == '\x03':
                break
            c = cv2.waitKey(3)
            if c == ord('q'):
                break
            if c == ord('a'):
                print(1)
                cv2.imwrite(str(j) + ".jpg", frame)
                j += 1

def main(args=None):
    rclpy.init(args=args)
    node = RacecarNode()
    try:
        node.Camera_vision()
    finally:
        end_time = time.time()
        run_time = end_time - node.start_time
        print(f"程序运行时间: {run_time:.2f} 秒")
        print("times:" + str(node.times))
        node.twist.linear.x = 0.0
        node.twist.angular.z = 0.0
        node.publisher_.publish(node.twist)
        cv2.waitKey(500)
        cv2.destroyAllWindows()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
