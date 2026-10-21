from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('gn10_navigation'))
    yaml_path = share / 'config/shooting_points.yaml'

    arguments = [
        DeclareLaunchArgument(
            'yaml_file_path',
            default_value=str(yaml_path),
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

