import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(bringup_dir, 'launch')
        
    ekf_config = Path(get_package_share_directory('racecar'), 'config', 'ekf.yaml')

    ekf_carto_config = Path(get_package_share_directory('racecar'), 'config', 'ekf_carto.yaml')
    carto_slam = LaunchConfiguration('carto_slam', default='false')
    carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')

    # imu
    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('hipnuc_imu')
    # 声明参数
    imu_package_arg = DeclareLaunchArgument(
        'imu_package', default_value='spec',
        description='package type [spec, 0x91]'
    )

    imu_package = LaunchConfiguration('imu_package')
            
     
    robot_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'ekf.launch.py')),
        launch_arguments={'carto_slam':carto_slam}.items(),            
    )
                                                            
        
    # imu_filter_node =  launch_ros.actions.Node(
    #     package='hipnuc_imu',
    #     executable='imu_filter_node',
    #     name='imu_filter_node',
    # )

        
                           
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher', 
        executable='joint_state_publisher', 
        name='joint_state_publisher',
    )


    return LaunchDescription([
        robot_ekf,
        carto_slam_dec,joint_state_publisher_node,imu_package_arg,
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lslidar_driver_share_dir, 'launch', 'lslidar_launch.py')
            )
        ),

        # Include IMU launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(imu_launch_share_dir, 'launch', 'imu_'), imu_package, '_msg.launch.py']
            )
        ),

        # imu_filter_node,

        Node(
            package='encoder',
            executable='encoder_node',
            name='encoder_vel',
            output='screen',

        ),
        Node(
            package='racecar_driver',
            executable='racecar_driver_node_one',
            name='racecar_driver',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser_link',
            arguments=['--x', '0.07', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'laser_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=['--x', '0.1653', '--y', '0.0', '--z', '0.0', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_footprint', '--child-frame-id', 'IMU_link']
        )

    ])

