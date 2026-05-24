from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包共享目录
    lslidar_driver_share_dir = get_package_share_directory('lslidar_driver')
    imu_launch_share_dir = get_package_share_directory('hipnuc_imu')
    
    # 声明参数
    imu_package_arg = DeclareLaunchArgument(
        'imu_package', default_value='spec',
        description='package type [spec, 0x91]'
    )

    imu_package = LaunchConfiguration('imu_package')

    return LaunchDescription([
        imu_package_arg,

        # Include Lidar launch
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

        # Encoder Driver Node
        Node(
            package='encoder',
            executable='encoder_node',
            name='encoder_vel',
            output='screen',
            parameters=[
                {'serial_port': '/dev/encoder'},
                {'k': 0.88},
                {'baud_rate': 57600}
            ]
        ),

        # Static Transform Publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint2base_link',
            arguments=['0.0', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2laser_link',
            arguments=['0.07', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link2imu',
            arguments=['0.1653', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'IMU_link']
        )
    ])