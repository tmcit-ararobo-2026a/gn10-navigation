#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
debug_shooting_points_planner.launch.py
---------------------------------------
自己位置推定や他ノードが動いていない単体環境下で、
shooting-point の目標地点 YAML ファイル (red / blue) を読み込み、
costmap_generator_node + path_planner_node と連携して各目的地間のパスを計画・表示するデバッグ用 Launch。
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('gn10_navigation')

    default_costmap_params = os.path.join(pkg_share, 'config', 'costmap_params.yaml')
    default_planner_params = os.path.join(pkg_share, 'config', 'path_planner_params.yaml')
    default_rviz_config    = os.path.join(pkg_share, 'rviz', 'debug_shooting_points_planner.rviz')

    # Launch 引数の定義
    zone_arg = DeclareLaunchArgument(
        'zone',
        default_value='blue',
        description='Team zone: "blue" or "red"'
    )

    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='tour',
        description='Planning mode: "tour" (sequential loop), "from_start" (from StartZone to all), "all_pairs", or "both"'
    )

    shooting_points_file_arg = DeclareLaunchArgument(
        'shooting_points_file',
        default_value='',
        description='Optional direct path to shooting points YAML file (empty for auto-discovery)'
    )

    costmap_params_arg = DeclareLaunchArgument(
        'costmap_params_file',
        default_value=default_costmap_params,
        description='Full path to costmap generator parameters YAML file'
    )

    planner_params_arg = DeclareLaunchArgument(
        'planner_params_file',
        default_value=default_planner_params,
        description='Full path to path planner parameters YAML file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz2 for visualization'
    )

    # 1. コストマップ生成ノード (静的マップ nhk2026_map.json から /costmap を生成)
    costmap_node = Node(
        package='gn10_navigation',
        executable='costmap_generator_node',
        name='costmap_generator_node',
        output='screen',
        parameters=[LaunchConfiguration('costmap_params_file')]
    )

    # 2. 経路計画ノード (path_planner_node)
    path_planner_node = Node(
        package='gn10_navigation',
        executable='path_planner_node',
        name='path_planner_node',
        output='screen',
        parameters=[LaunchConfiguration('planner_params_file')]
    )

    # 3. シューティングポイント パス計画デバッグノード
    debugger_node = Node(
        package='gn10_navigation',
        executable='debug_shooting_points_planner.py',
        name='debug_shooting_points_planner',
        output='screen',
        parameters=[{
            'zone': LaunchConfiguration('zone'),
            'mode': LaunchConfiguration('mode'),
            'shooting_points_file': LaunchConfiguration('shooting_points_file'),
            'map_frame': 'map',
            'base_frame': 'base_link'
        }]
    )

    # 4. RViz2 (可視化)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='debug_planner_rviz',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        zone_arg,
        mode_arg,
        shooting_points_file_arg,
        costmap_params_arg,
        planner_params_arg,
        rviz_arg,
        costmap_node,
        path_planner_node,
        debugger_node,
        rviz_node
    ])
