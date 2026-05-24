#!/usr/bin/env python3
"""
激光雷达循迹ROS2包启动文件
功能：启动激光雷达循迹节点，并从参数文件加载参数
作者：zyh
版本：1.1
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('lidar_tracking')
    params_file = os.path.join(pkg_share, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='lidar_tracking',                    # 包名
            executable='lidar_tracking',                 # 可执行文件名
            name='lidar_tracking',                       # 运行时节点名
            output='screen',
            parameters=[params_file]                     # 从YAML加载参数
            # 如需在此基础上小范围覆盖，可追加一个字典作末端覆盖：
            # parameters=[params_file, {'/**': {'ros__parameters': {'rate2': 2.4}}}]
        )
    ])