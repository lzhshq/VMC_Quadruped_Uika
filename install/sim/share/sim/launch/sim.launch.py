#!/usr/bin/env python3
"""
sim.launch.py - MuJoCo 仿真启动文件

功能：
- 启动 MuJoCo 仿真节点
- 加载仿真参数
- 可选：启动 robot_state_publisher
- 可选：启动 RViz2

使用方法:
  ros2 launch sim sim.launch.py

作者: UIKA Team
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """生成 launch 描述"""

    # 包名
    pkg_name = 'sim'

    # 配置文件路径
    sim_params_file = os.path.join(
        get_package_share_directory(pkg_name),
        'config', 'sim_params.yaml'
    )

    # ============================================================
    # Launch 参数声明
    # ============================================================
    use_viewer_arg = DeclareLaunchArgument(
        'use_viewer',
        default_value='true',
        description='是否显示 MuJoCo 可视化窗口'
    )

    use_rsp_arg = DeclareLaunchArgument(
        'use_robot_state_publisher',
        default_value='false',
        description='是否启动 robot_state_publisher'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动 RViz2'
    )

    # ============================================================
    # MuJoCo 仿真节点
    # ============================================================
    mujoco_sim_node = Node(
        package=pkg_name,
        executable='mujoco_sim_node',
        name='mujoco_sim_node',
        output='screen',
        parameters=[sim_params_file],
        emulate_tty=True,
    )

    # ============================================================
    # Robot State Publisher（可选）
    # ============================================================
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        condition=IfCondition(LaunchConfiguration('use_robot_state_publisher')),
        parameters=[{
            'robot_description': '',
            'use_sim_time': True,
        }],
    )

    # ============================================================
    # RViz2（可选）
    # ============================================================
    rviz_config_file = os.path.join(
        get_package_share_directory('description'),
        'config', 'display.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    # ============================================================
    # 构建 LaunchDescription
    # ============================================================
    ld = LaunchDescription()

    # 添加参数
    ld.add_action(use_viewer_arg)
    ld.add_action(use_rsp_arg)
    ld.add_action(use_rviz_arg)

    # 添加节点
    ld.add_action(mujoco_sim_node)
    ld.add_action(rsp_node)
    ld.add_action(rviz_node)

    return ld
