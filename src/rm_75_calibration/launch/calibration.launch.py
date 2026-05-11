#!/usr/bin/env python3
"""
标定启动文件
- force_sensor_node: 六维力传感器驱动（200Hz 发布 /force_sensor_data）
- force_control_node: 多轴恒力控制节点（步进试探式）

参数统一管理: config/force_control_params.yaml

用法:
  ros2 launch rm_75_calibration calibration.launch.py
  ros2 launch rm_75_calibration calibration.launch.py control_axes:=xz
  ros2 launch rm_75_calibration calibration.launch.py control_axes:=xyz z_target_force:=15.0 x_target_force:=5.0
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('rm_75_calibration')
    yaml_path = os.path.join(pkg_share, 'config', 'force_control_params.yaml')

    # 读取 YAML 获取默认值，用于命令行覆参的 default_value
    with open(yaml_path, 'r') as f:
        yaml_defaults = yaml.safe_load(f)['/force_control_node']['ros__parameters']

    # =========================================================================
    # 高频命令行覆参（仅保留最常用的 5 个，其余参数改 YAML 文件）
    # =========================================================================
    control_axes_arg = DeclareLaunchArgument(
        'control_axes', default_value=str(yaml_defaults['control_axes']),
        description='控制轴组合: z / xz / yz / xyz')
    z_target_arg = DeclareLaunchArgument(
        'z_target_force', default_value=str(yaml_defaults['z_target_force']),
        description='Z 轴目标力值 (N)')
    x_target_arg = DeclareLaunchArgument(
        'x_target_force', default_value=str(yaml_defaults['x_target_force']),
        description='X 轴目标力值 (N)')
    y_target_arg = DeclareLaunchArgument(
        'y_target_force', default_value=str(yaml_defaults['y_target_force']),
        description='Y 轴目标力值 (N)')
    hold_duration_arg = DeclareLaunchArgument(
        'hold_duration', default_value=str(yaml_defaults['hold_duration']),
        description='保压时长 (s)，0=无限保压')

    # =========================================================================
    # 节点参数: YAML 基础值 + 命令行覆写（后面的覆盖前面的）
    # =========================================================================
    node_params = [
        yaml_path,
        {
            'control_axes': LaunchConfiguration('control_axes'),
            'z_target_force': LaunchConfiguration('z_target_force'),
            'x_target_force': LaunchConfiguration('x_target_force'),
            'y_target_force': LaunchConfiguration('y_target_force'),
            'hold_duration': LaunchConfiguration('hold_duration'),
        },
    ]

    return LaunchDescription([
        control_axes_arg,
        z_target_arg,
        x_target_arg,
        y_target_arg,
        hold_duration_arg,
        Node(
            package='rm_75_calibration',
            executable='force_sensor_node',
            name='force_sensor_node',
            output='screen',
        ),
        Node(
            package='rm_75_calibration',
            executable='force_control_node',
            name='force_control_node',
            output='screen',
            parameters=node_params,
        ),
    ])
