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
    default_params_file = os.path.join(pkg_share, 'config', 'path_planner_params.yaml')

    # Launch 引数の定義
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file to use'
    )

    # Path Planner ノードの設定
    path_planner_node = Node(
        package='gn10_navigation',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file')
        ]
    )

    return LaunchDescription([
        params_file_arg,
        path_planner_node
    ])