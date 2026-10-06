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
    default_params_file = os.path.join(pkg_share, 'config', 'costmap_params.yaml')

    # Launch 引数の定義
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters YAML file to use'
    )

    # Costmap Generator ノードの設定
    costmap_node = Node(
        package='gn10_navigation',
        executable='costmap_generator_node',
        name='costmap_generator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file')
        ]
    )

    return LaunchDescription([
        params_file_arg,
        costmap_node
    ])