from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
#ros2 racecar xunxian.launch.py
def generate_launch_description():
    # 摄像头启动文件路径
    camera_launch = os.path.join(
        get_package_share_directory('racecar'),
        'launch',
        'camera.launch.py'
    )

    # 启动摄像头
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch)
    )

    # 启动视觉巡线节点
    line_follow_node = Node(
        package='racecar',
        executable='line_follow',
        name='line_follow',
        output='screen'
    )

    return LaunchDescription([
        camera,
        line_follow_node
    ])