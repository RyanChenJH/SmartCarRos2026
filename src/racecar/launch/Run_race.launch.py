import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    racecar_dir = get_package_share_directory('racecar')
    launch_dir = os.path.join(racecar_dir, 'launch')

    map_file = LaunchConfiguration('map')
    nav_params_file = LaunchConfiguration('params')
    race_params_file = LaunchConfiguration('race_params')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    default_map = os.path.join(racecar_dir, 'map', 'ai_map.yaml')
    default_nav_params = os.path.join(racecar_dir, 'config', 'nav.yaml')
    default_race_params = os.path.join(racecar_dir, 'config', 'race_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the race map yaml file'),
        DeclareLaunchArgument(
            'params',
            default_value=default_nav_params,
            description='Full path to the Nav2 parameter file'),
        DeclareLaunchArgument(
            'race_params',
            default_value=default_race_params,
            description='Full path to the race manager parameter file'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav_params_file,
                'slam': 'False',
            }.items(),
        ),

        Node(
            package='racecar',
            executable='race_manager.py',
            name='race_manager',
            output='screen',
            parameters=[race_params_file],
        ),
    ])
