from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start_target_manager(context):
    share = Path(get_package_share_directory('gn10_navigation'))
    zone = LaunchConfiguration('zone').perform(context).strip().lower()
    yaml_path = LaunchConfiguration('yaml_file_path').perform(context).strip()

    if not yaml_path:
        if zone == 'red':
            yaml_path = str(share / 'config/shooting_points_red.yaml')
        else:
            yaml_path = str(share / 'config/shooting_points_blue.yaml')

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
            description='Path to shooting_points.yaml (if omitted, chosen by zone)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
    ]

    return LaunchDescription(arguments + [OpaqueFunction(function=_start_target_manager)])

