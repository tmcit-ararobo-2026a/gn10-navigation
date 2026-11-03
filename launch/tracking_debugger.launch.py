import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    debugger_node = Node(
        package='gn10_navigation',
        executable='tracking_debugger_gui.py',
        name='tracking_debugger_gui',
        output='screen'
    )

    return LaunchDescription([
        debugger_node
    ])
