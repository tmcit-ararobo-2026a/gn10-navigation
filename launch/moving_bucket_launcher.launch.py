from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base link frame id'
        ),
        DeclareLaunchArgument(
            'launch_angle_deg',
            default_value='45.0',
            description='Belt launcher muzzle elevation angle [deg]'
        ),
        DeclareLaunchArgument(
            'min_velocity',
            default_value='3.0',
            description='Minimum allowable launch velocity [m/s]'
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='7.75',
            description='Maximum allowable launch velocity [m/s]'
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
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'launch_angle_deg': LaunchConfiguration('launch_angle_deg'),
                'min_velocity': LaunchConfiguration('min_velocity'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
