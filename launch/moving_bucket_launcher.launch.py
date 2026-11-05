from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('gn10_navigation'))
    default_params = str(share / 'config/moving_bucket_launcher_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launcher_params',
            default_value=default_params,
            description='Path to moving_bucket_launcher_node parameter YAML file'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        Node(
            package='gn10_navigation',
            executable='moving_bucket_launcher_node',
            name='moving_bucket_launcher_node',
            output='screen',
            parameters=[
                LaunchConfiguration('launcher_params'),
                {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
            ],
        ),
    ])
