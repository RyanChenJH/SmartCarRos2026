#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import csv
import codecs

class GoalPoseSubscriber(Node):
    def __init__(self):
        super().__init__('get_point')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.pose_stamped_cb,
            10)
        self.subscription  # prevent unused variable warning
        self.points = []
        self.settings = termios.tcgetattr(sys.stdin)

    def pose_stamped_cb(self, msg):
        gx = msg.pose.position.x
        gy = msg.pose.position.y
        gz = msg.pose.orientation.z
        gw = msg.pose.orientation.w
        self.points.append([gx, gy, gz, gw])

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def data_write_csv(self, file_name, datas):
        with codecs.open(file_name, 'w+', 'utf-8') as file_csv:
            writer = csv.writer(file_csv, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            for data in datas:
                writer.writerow(data)
        self.get_logger().info('Data written to CSV successfully!')

def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = node.get_key()
            if key == 'f' or key == 'F':
                node.data_write_csv('/home/davinci-mini/CHEN2/src/racecar/scripts/out_test.csv', node.points)
            if key == '\x03':  # Ctrl+C to exit
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()