import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import threading
import time

class VideoRecorder(Node):
    def __init__(self):
        super().__init__('video_recorder')

        # ���������ߣ�����'/camera1/image_raw'����
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.subscription  # ��ֹδʹ�ñ����ľ���

        # ʹ��CvBridge��ROSͼ����Ϣת��ΪOpenCVͼ��
        self.bridge = CvBridge()

        # ��Ƶ��д��
        self.video_writer = None

        # ����¼���߳��˳��ı�־
        self.recording = True

        # ����¼�ƶ�ʱ���߳�
        self.timer_thread = threading.Thread(target=self.stop_recording_after_delay, args=(10,))
        self.timer_thread.start()

    def image_callback(self, msg):
        if not self.recording:
            return

        # ��ROSͼ����Ϣת��ΪOpenCVͼ��
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # �����Ƶ��д��δ�������򴴽���
        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter('output.avi', fourcc, 30.0, (frame.shape[1], frame.shape[0]))

        # д��֡���ݵ���Ƶ�ļ�
        self.video_writer.write(frame)

    def stop_recording_after_delay(self, delay_seconds):
        # �ȴ�ָ����ʱ��
        time.sleep(delay_seconds)
        self.recording = False
        self.get_logger().info('Recording stopped after {} seconds.'.format(delay_seconds))

        # �ر���Ƶ��д��
        if self.video_writer is not None:
            self.video_writer.release()

        # �رսڵ�
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    video_recorder = VideoRecorder()

    rclpy.spin(video_recorder)

    video_recorder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()