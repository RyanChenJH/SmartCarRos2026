import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from action_msgs.msg import GoalStatus,GoalStatusArray

class waypoint_cycle(Node):
    def __init__(self):
        super().__init__('waypoint_cycle')
        self.markerArray = MarkerArray()
        self.markerArray_number = MarkerArray()
        self.count = 0
        self.index = 0
        self.try_again = True

        self.mark_pub = self.create_publisher(MarkerArray, '/path_point', 100)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)
        self.navigation_goal_status_sub_ = self.create_subscription(GoalStatusArray,"navigate_to_pose/_action/status",self.pose_callback,10)


    def pose_callback(self, msg):
        # self.get_logger().info(f'status:{msg.status_list[-1].status},count:{self.count}')
        if msg.status_list[-1].status != GoalStatus.STATUS_EXECUTING:
            if msg.status_list[-1].status >= GoalStatus.STATUS_SUCCEEDED and self.count > 0:
                self.try_again = True
                if self.index == self.count:
                    self.print_marker_info(self.index-1)
                    if self.count > 1:
                        self.get_logger().info('Complete instructions!')
                    self.index = 0
                    self.publish_pose(self.index)
                    self.index += 1
                elif self.index < self.count:
                    self.get_logger().info(f'Reach the target point {self.index-1}:')
                    self.print_marker_info(self.index-1)
                    self.publish_pose(self.index)
                    self.index += 1
            elif self.count > 0:
                self.get_logger().warn(f'Can not reach the target point {self.index-1}:')
                self.print_marker_info(self.index-1)
                if self.try_again:
                    self.get_logger().warn(f'trying reach the target point {self.index-1} again!')
                    self.publish_pose(self.index - 1)
                    self.try_again = False
                elif self.index < len(self.markerArray.markers):
                    self.get_logger().warn(f'try reach the target point {self.index-1} failed! reach next point:')
                    self.print_marker_info(self.index-1)
                    if self.index == self.count:
                        self.index = 0
                    self.publish_pose(self.index)
                    self.index += 1
                    self.try_again = True

    def click_callback(self, msg):
        self.add_marker(msg.point.x, msg.point.y, 0.0, 1.0)
        self.add_marker_number(msg.point.x, msg.point.y, 0.0, 1.0)
        self.mark_pub.publish(self.markerArray)
        self.mark_pub.publish(self.markerArray_number)
        if self.count == 0:
            self.publish_pose(self.count)
            self.index += 1
        self.count += 1

    def add_marker(self, x, y, z, w):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.ns = 'arrow'
        marker.id = self.count
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        self.markerArray.markers.append(marker)

    def add_marker_number(self, x, y, z, w):
        marker_number = Marker()
        marker_number.header.frame_id = 'map'
        marker_number.ns = 'number'
        marker_number.id = self.count
        marker_number.type = Marker.TEXT_VIEW_FACING
        marker_number.action = Marker.ADD
        marker_number.scale.x = 0.5
        marker_number.scale.y = 0.5
        marker_number.scale.z = 0.5
        marker_number.color.a = 1.0
        marker_number.color.r = 1.0
        marker_number.pose.position.x = x
        marker_number.pose.position.y = y
        marker_number.pose.position.z = 0.1
        marker_number.pose.orientation.z = z
        marker_number.pose.orientation.w = w
        marker_number.text = str(self.count)
        self.markerArray_number.markers.append(marker_number)

    def publish_pose(self, index):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.markerArray.markers[index].pose.position.x
        pose.pose.position.y = self.markerArray.markers[index].pose.position.y
        pose.pose.orientation.z = self.markerArray.markers[index].pose.orientation.z
        pose.pose.orientation.w = self.markerArray.markers[index].pose.orientation.w
        self.goal_pub.publish(pose)

    def print_marker_info(self, index):
        self.get_logger().info(
            f"x: {self.markerArray.markers[index].pose.position.x}, "
            f"y: {self.markerArray.markers[index].pose.position.y}, "
            f"z: {self.markerArray.markers[index].pose.orientation.z}, "
            f"w: {self.markerArray.markers[index].pose.orientation.w}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = waypoint_cycle()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
