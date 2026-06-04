#!/usr/bin/env python3
"""
标定启动文件
- force_sensor_node: 六维力传感器驱动（200Hz 发布 /force_sensor_data）
- force_control_node: 多轴恒力控制节点（网格遍历模式）

参数统一管理: config/force_control_params.yaml

用法:
  ros2 launch rm_75_calibration calibration.launch.py
  ros2 launch rm_75_calibration calibration.launch.py control_axes:=xz
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
    # 命令行覆参
    # =========================================================================
    control_axes_arg = DeclareLaunchArgument(
        'control_axes', default_value=str(yaml_defaults['control_axes']),
        description='控制轴组合: z / xz / yz / xyz')

    # =========================================================================
    # 节点参数: YAML 基础值 + 命令行覆写
    # =========================================================================
    node_params = [
        yaml_path,
        {
            'control_axes': LaunchConfiguration('control_axes'),
        },
    ]

    # 实时显示脚本路径（YAML 中 realtime_script 参数，'none' 禁用）
    realtime_dir = yaml_defaults.get('realtime_script', 'none')

    items = [
        control_axes_arg,
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
    ]

    if realtime_dir != 'none':
        items.append(
            ExecuteProcess(
                cmd=['python3', 'main.py'],
                cwd=realtime_dir,
                output='screen',
            )
        )

    return LaunchDescription(items)
