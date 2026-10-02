import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # パッケージの share ディレクトリを取得
    pkg_share = get_package_share_directory('gn10_navigation')

    # デフォルトの YAML パラメータファイルのパス
    default_costmap_params = os.path.join(pkg_share, 'config', 'costmap_params.yaml')
    default_planner_params = os.path.join(pkg_share, 'config', 'path_planner_params.yaml')

    # Launch 引数の定義
    costmap_params_arg = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=default_costmap_params,
        description='Full path to the costmap generator parameters YAML file'
    )

    planner_params_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value=default_planner_params,
        description='Full path to the path planner parameters YAML file'
    )

    # Costmap Generator ノードの設定
    costmap_node = Node(
        package='gn10_navigation',
        executable='costmap_generator_node',
        name='costmap_generator_node',
        output='screen',
        parameters=[LaunchConfiguration('costmap_params_file')]
    )

    # Path Planner ノードの設定
    path_planner_node = Node(
        package='gn10_navigation',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('planner_params_file')]
    )

    return LaunchDescription([
        costmap_params_arg,
        planner_params_arg,
        costmap_node,
        path_planner_node
    ])