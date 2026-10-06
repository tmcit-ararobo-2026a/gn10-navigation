"""別途起動した自己位置推定の map -> base_link TF を利用する。"""

from pathlib import Path
import math

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_navigation(context):
    follower_parameters = [LaunchConfiguration('follower_params')]
    planner_parameters = [LaunchConfiguration('planner_params')]
    speed = LaunchConfiguration('max_speed').perform(context)
    if speed:
        speed = float(speed)
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError('max_speed must be finite and positive [m/s]')
        follower_parameters.append({'max_speed': speed})
    clock = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    follower_parameters.append(clock)
    planner_parameters.append(clock)
    rviz_config = LaunchConfiguration('rviz_config').perform(context)
    nodes = [
        Node(package='gn10_navigation', executable='costmap_generator_node',
             name='costmap_generator_node', output='screen',
             parameters=[LaunchConfiguration('costmap_params'), clock,
                         {'map_file_path': ParameterValue(
                             LaunchConfiguration('map_file_path'), value_type=str)}]),
        Node(package='gn10_navigation', executable='path_planner_node',
             name='path_planner_node', output='screen', parameters=planner_parameters),
        Node(package='gn10_navigation', executable='path_follower_node',
             name='path_follower_node', output='screen',
             parameters=follower_parameters),
        Node(package='rviz2', executable='rviz2', name='navigation_rviz',
             output='screen', arguments=['-d', rviz_config] if rviz_config else [],
             parameters=[clock], condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Navigation process exited'))],
    )) for node in nodes]
    return handlers + nodes


def generate_launch_description():
    share = Path(get_package_share_directory('gn10_navigation'))
    rviz_config = share / 'rviz/navigation.rviz'
    arguments = [
        DeclareLaunchArgument('use_rviz', default_value='true', description='RViz2 を起動する'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='実機は false、rosbag の /clock 利用時は true'),
        DeclareLaunchArgument('planner_params',
                              default_value=str(share / 'config/path_planner_params.yaml')),
        DeclareLaunchArgument('follower_params',
                              default_value=str(share / 'config/path_follower_params.yaml')),
        DeclareLaunchArgument('max_speed', default_value='',
                              description='最高並進速度 [m/s]。空欄なら follower_params の値'),
        DeclareLaunchArgument('costmap_params',
                              default_value=str(share / 'config/costmap_params.yaml')),
        DeclareLaunchArgument('map_file_path',
                              default_value=str(share / 'config/nhk2026_map.json'),
                              description='自己位置推定と同じフィールドの JSON マップ'),
        DeclareLaunchArgument('rviz_config',
                              default_value=str(rviz_config) if rviz_config.is_file() else '',
                              description='RViz設定ファイル。空欄なら標準表示で起動'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=start_navigation)])
