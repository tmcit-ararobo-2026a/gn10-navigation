import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_default_shooting_points_path() -> str:
    # 1. Environment variable
    env_path = os.environ.get('GN10_SHOOTING_POINTS_PATH')
    if env_path and Path(env_path).exists():
        return str(Path(env_path).resolve())

    # 2. data/shooting-points repository
    candidates = [
        Path.home() / 'ros2_ws/data/shooting-points/shooting_points.yaml',
        Path.home() / 'ros2_ws/data/shooting-points/shooting_point.yaml',
        Path('data/shooting-points/shooting_points.yaml').resolve(),
    ]
    for c in candidates:
        if c.exists():
            return str(c)

    # 3. Package share directory (fallback)
    share = Path(get_package_share_directory('gn10_navigation'))
    return str(share / 'config/shooting_points.yaml')


def generate_launch_description():
    default_yaml_path = get_default_shooting_points_path()

    arguments = [
        DeclareLaunchArgument(
            'yaml_file_path',
            default_value=default_yaml_path,
            description='Path to shooting_points.yaml'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
    ]

    node = Node(
        package='gn10_navigation',
        executable='target_manager_node',
        name='target_manager_node',
        output='screen',
        parameters=[{
            'yaml_file_path': LaunchConfiguration('yaml_file_path'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription(arguments + [node])

