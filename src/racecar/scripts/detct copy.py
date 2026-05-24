import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist

line_flag = False

class Racecar_opencv(Node):
    def __init__(self):
        super().__init__('racecar_opencv')
        self.publisher_cmd = self.create_publisher(Twist, '/car_cmd_vel', 10)
        # self.timer_period = 0.5  # seconds
        # self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.msg = Twist()
        self.msg.linear.x = float(1525)
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z =float(90)


    def create_mask(self,picture, mask_point):
        """ 创建掩模函数 """
        mask = np.zeros_like(picture)
        cv2.fillPoly(mask, mask_point, 255)
        mask_img = cv2.bitwise_and(picture, mask)
        return mask_img

    def calculate_k_and_b(self,contour):
        """ 根据轮廓计算斜率 k 和截距 b """
        rows, cols = [480, 640]
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        k = vy / vx
        b = y - k * x
        return k, b

    def angle_calculate(self,k, b, frame):

        """ 根据斜率和截距计算角度 """
        y0 = 240  # 屏幕中心纵坐标
        x0 = k * y0 + b  # 计算横坐标
        delta_y = frame.shape[0] - y0  # 计算y的变化量
        delta_x = x0 - frame.shape[1] / 2  # 计算x的变化量
        angle = -np.arctan2(delta_x, delta_y) * 180 / np.pi  # 计算角度
        # if angle < 40:
        #     angle = 40
        # if angle > 80:
             
        # print ("angle:", angle)
        print ("flag:", line_flag)

        if line_flag == False:
            angle =  102
        else:
            angle = 68
        self.msg.angular.z = float(angle)
        if self.publisher_cmd is not None:
            self.publisher_cmd.publish(self.msg)
        
        return angle

    def process_frame(self,frame, lower_yellow, upper_yellow):
        """ 处理每一帧图像以检测黄色车道线和计算角度 """
        mask_point = np.array([[(0, 0), (640, 0), (640, 480), (0, 480)]])  # 道路掩模区域设置
        
        # 将图像转换为HSV颜色空间
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # 创建黄色颜色范围的掩模
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # 对黄色掩模应用道路掩模
        masked_yellow = self.create_mask(yellow_mask, mask_point)
        
        # 对黄色掩模进行边缘检测
        edges = cv2.Canny(masked_yellow, 50, 150)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 60:  # 简单的面积过滤
                k, b = self.calculate_k_and_b(contour)
                angle = self.angle_calculate(k, b, frame)
                print("Detected angle:", k)
                # M = cv2.moments(contour)
                # if M["m00"] == 0:  # 避免除以零错误
                #     cx = cy = 0
                # else:
                #     cx = int(M['m10'] / M['m00'])
                #     cy = int(M['m01'] / M['m00'])
                # if cx > frame.shape[1] // 2:
                #     line_flag = True
                # else:
                #     line_flag = False
                if k < 0:
                    line_flag = False
                else:
                    line_flag = True

        
        # cv2.imshow('Yellow Lanes', masked_yellow)
        # cv2.imshow('Edges', edges)

        # def on_trackbar(val):
        #     """ 滑动块回调函数 """
        #     pass

def main(args=None):
    rclpy.init(args=args)

if __name__ == '__main__':

    main()
    # 创建滑动块窗口
    # cv2.namedWindow('Yellow Range', cv2.WINDOW_NORMAL)

    # cv2.createTrackbar('Hue Low', 'Yellow Range', 20, 179, on_trackbar)
    # cv2.createTrackbar('Hue High', 'Yellow Range', 30, 179, on_trackbar)
    # cv2.createTrackbar('Sat Low', 'Yellow Range', 100, 255, on_trackbar)
    # cv2.createTrackbar('Sat High', 'Yellow Range', 255, 255, on_trackbar)
    # cv2.createTrackbar('Val Low', 'Yellow Range', 100, 255, on_trackbar)
    # cv2.createTrackbar('Val High', 'Yellow Range', 255, 255, on_trackbar)

    # # 初始化滑动块位置
    # cv2.setTrackbarPos('Hue Low', 'Yellow Range', 20)
    # cv2.setTrackbarPos('Hue High', 'Yellow Range', 30)
    # cv2.setTrackbarPos('Sat Low', 'Yellow Range', 100)
    # cv2.setTrackbarPos('Sat High', 'Yellow Range', 255)
    # cv2.setTrackbarPos('Val Low', 'Yellow Range', 100)
    # cv2.setTrackbarPos('Val High', 'Yellow Range', 255)

    # 打开摄像头
    cap = cv2.VideoCapture(0)
    node = Racecar_opencv()
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        # hue_low = cv2.getTrackbarPos('Hue Low', 'Yellow Range')
        # hue_high = cv2.getTrackbarPos('Hue High', 'Yellow Range')
        # sat_low = cv2.getTrackbarPos('Sat Low', 'Yellow Range')
        # sat_high = cv2.getTrackbarPos('Sat High', 'Yellow Range')
        # val_low = cv2.getTrackbarPos('Val Low', 'Yellow Range')
        # val_high = cv2.getTrackbarPos('Val High', 'Yellow Range')
        
        lower_yellow = np.array([21, 66, 115])
        upper_yellow = np.array([179, 255, 255])
        
        node.process_frame(frame,lower_yellow,upper_yellow)
        
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break
    cap.release()
    rclpy.shutdown()