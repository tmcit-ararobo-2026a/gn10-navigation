import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('gn10_navigation')

    default_planner_params = os.path.join(pkg_share, 'config', 'path_planner_params.yaml')

    planner_params_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value=default_planner_params,
        description='Full path to the path planner parameters YAML file'
    )

    path_planner_node = Node(
        package='gn10_navigation',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('planner_params_file')]
    )

    return LaunchDescription([
        planner_params_arg,
        path_planner_node
    ])