from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    hi_puc_share_dir = FindPackageShare('hipnuc_imu').find('hipnuc_imu')

    # Include the rf2o_Laser_Odometry launch file
    # rf2o_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution([hi_puc_share_dir, 'launch', 'includes', 'rf2o.launch.py'])])
    # )

    # EKF localization node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_se',
        parameters=[PathJoinSubstitution([hi_puc_share_dir, 'param', 'ekf_params.yaml'])],
        output='screen'
    )

    # gmapping node
    gmapping_node = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true', description='Use RViz for visualization'),

        ekf_node,
        gmapping_node
    ])