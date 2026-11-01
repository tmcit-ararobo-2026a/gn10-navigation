import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_default_shooting_points_path(zone: str = 'blue') -> str:
    # 1. Environment variable
    env_path = os.environ.get('GN10_SHOOTING_POINTS_PATH')
    if env_path and Path(env_path).exists():
        return str(Path(env_path).resolve())

    zone = zone.strip().lower()
    filenames = []
    if zone == 'red':
        filenames = ['shooting_points_red.yaml', 'shooting_point_red.yaml']
    else:
        filenames = ['shooting_points_blue.yaml', 'shooting_point_blue.yaml', 'shooting_points.yaml', 'shooting_point.yaml']

    # 2. data/shooting-points repository
    for filename in filenames:
        candidates = [
            Path.home() / 'ros2_ws/data/shooting-points' / filename,
            Path(f'data/shooting-points/{filename}').resolve(),
            Path.home() / 'ros2_ws/data/shooting-point-config' / filename,
        ]
        for c in candidates:
            if c.exists():
                return str(c)

    # 3. Package share directory (fallback)
    share = Path(get_package_share_directory('gn10_navigation'))
    if zone == 'red':
        red_path = share / 'config/shooting_points_red.yaml'
        if red_path.exists():
            return str(red_path)
    else:
        blue_path = share / 'config/shooting_points_blue.yaml'
        if blue_path.exists():
            return str(blue_path)

    return str(share / 'config/shooting_points.yaml')


def _start_target_manager(context):
    zone = LaunchConfiguration('zone').perform(context).strip().lower()
    yaml_path = LaunchConfiguration('yaml_file_path').perform(context).strip()

    if not yaml_path:
        yaml_path = get_default_shooting_points_path(zone)

    return [
        Node(
            package='gn10_navigation',
            executable='target_manager_node',
            name='target_manager_node',
            output='screen',
            parameters=[{
                'yaml_file_path': yaml_path,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        )
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument(
            'zone',
            default_value='blue',
            description='Field zone ("blue" or "red")'
        ),
        DeclareLaunchArgument(
            'yaml_file_path',
            default_value='',
            description='Path to shooting_points.yaml (if omitted, chosen by zone and repository lookup)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_start_target_manager)])
