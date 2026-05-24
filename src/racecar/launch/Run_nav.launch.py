import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

        
    # nav_dir = get_package_share_directory('racecar')
    # nav_launchr = os.path.join(nav_dir, 'launch')

    racecar_dir = get_package_share_directory('racecar')
    racecar_launchr = os.path.join(racecar_dir, 'launch')

    map_dir = os.path.join(racecar_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(
        map_dir, 'ai_map.yaml'))
 
    param_dir = os.path.join(racecar_dir, 'config')
    param_file = LaunchConfiguration('params', default=os.path.join(
        param_dir, 'nav.yaml'))


    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'params',
            default_value=param_file,
            description='Full path to param file to load'),
        Node(
            name='waypoint_cycle',
            package='nav2_waypoint_cycle',
            executable='nav2_waypoint_cycle',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [racecar_launchr, '/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': param_file}.items(),
        ),

    ])
