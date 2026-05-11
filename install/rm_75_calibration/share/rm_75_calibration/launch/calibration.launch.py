#!/usr/bin/env python3
"""
标定启动文件
- force_sensor_node: 六维力传感器驱动（200Hz 发布 /force_sensor_data）
- force_control_node: 多轴恒力控制节点（步进试探式）

用法:
  ros2 launch rm_75_calibration calibration.launch.py
  ros2 launch rm_75_calibration calibration.launch.py control_axes:=xz
  ros2 launch rm_75_calibration calibration.launch.py control_axes:=xyz z_target_force:=15.0 x_target_force:=5.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # =========================================================================
    # 所有可调参数定义（通过命令行 := 覆盖，如 z_target_force:=15.0）
    # =========================================================================

    # ---------- 控制模式 ----------
    control_axes_arg = DeclareLaunchArgument(
        'control_axes', default_value='z',
        description='控制轴组合: x / y / z / xz / xy / xyz')

    # ---------- 机械臂初始位姿（MoveJp 关节空间目标） ----------
    start_x_arg = DeclareLaunchArgument(
        'start_x', default_value='0.432',
        description='初始 X 坐标 (m)')
    start_y_arg = DeclareLaunchArgument(
        'start_y', default_value='-0.0065',
        description='初始 Y 坐标 (m)')
    start_z_arg = DeclareLaunchArgument(
        'start_z', default_value='0.19',
        description='初始 Z 坐标 (m)')

    # ---------- 末端姿态（四元数，默认竖直向下） ----------
    ori_x_arg = DeclareLaunchArgument(
        'orientation_x', default_value='0.0',
        description='末端姿态 X 分量')
    ori_y_arg = DeclareLaunchArgument(
        'orientation_y', default_value='1.0',
        description='末端姿态 Y 分量')
    ori_z_arg = DeclareLaunchArgument(
        'orientation_z', default_value='0.0',
        description='末端姿态 Z 分量')
    ori_w_arg = DeclareLaunchArgument(
        'orientation_w', default_value='0.0',
        description='末端姿态 W 分量')

    # ---------- 运动参数 ----------
    movej_speed_arg = DeclareLaunchArgument(
        'movej_speed', default_value='20',
        description='MoveJp 关节空间规划速度')
    hold_duration_arg = DeclareLaunchArgument(
        'hold_duration', default_value='0.0',
        description='全部轴进入 HOLD 后的保压时间 (s)，0=无限保压直到手动停止')
    trajectory_duration_arg = DeclareLaunchArgument(
        'trajectory_duration', default_value='0.0',
        description='连续运动时长 (s)，>0 时忽略力控做匀速运动，0=步进试探模式')

    # =========================================================================
    # Z 轴参数（法向力 Fz，force_sign=-1 使向下为正）
    # =========================================================================
    z_target_arg = DeclareLaunchArgument(
        'z_target_force', default_value='10.0',
        description='Z 轴目标力值 (N)')
    z_step_arg = DeclareLaunchArgument(
        'z_step_size', default_value='0.00005',
        description='Z 轴精细步长 (m)，0.05mm')
    z_approach_arg = DeclareLaunchArgument(
        'z_approach_step_size', default_value='0.0001',
        description='Z 轴粗调步长 (m)，0.1mm')
    z_fine_ratio_arg = DeclareLaunchArgument(
        'z_fine_threshold_ratio', default_value='0.8',
        description='Z 轴粗→细切换比例（达到目标力×此值后切精细步长）')
    z_coarse_wait_arg = DeclareLaunchArgument(
        'z_coarse_stabilize_cycles', default_value='10',
        description='Z 轴粗调步间等待周期（100Hz 下 10=0.1s）')
    z_fine_wait_arg = DeclareLaunchArgument(
        'z_fine_stabilize_cycles', default_value='50',
        description='Z 轴精细步间等待周期（100Hz 下 50=0.5s）')
    z_drift_arg = DeclareLaunchArgument(
        'z_drift_threshold', default_value='0.2',
        description='Z 轴 HOLD 漂移阈值 (N)，超此值自动纠偏')
    z_sign_arg = DeclareLaunchArgument(
        'z_step_sign', default_value='1',
        description='Z 轴运动方向符号: 1 或 -1')
    z_delay_arg = DeclareLaunchArgument(
        'z_start_delay', default_value='0.0',
        description='Z 轴启动延迟 (s)，Z 为第一优先轴此值通常无效')
    z_field_arg = DeclareLaunchArgument(
        'z_force_field', default_value='force.z',
        description='Z 轴对应的 Wrench 字段: force.x / force.y / force.z')
    z_fsign_arg = DeclareLaunchArgument(
        'z_force_sign', default_value='-1',
        description='Z 轴力值符号翻转: -1=取反（向下为正），1=保持原符号')

    # =========================================================================
    # X 轴参数（切向力 Fx）
    # =========================================================================
    x_target_arg = DeclareLaunchArgument(
        'x_target_force', default_value='5.0',
        description='X 轴目标力值 (N)')
    x_step_arg = DeclareLaunchArgument(
        'x_step_size', default_value='0.00005',
        description='X 轴精细步长 (m)，0.05mm')
    x_approach_arg = DeclareLaunchArgument(
        'x_approach_step_size', default_value='0.0001',
        description='X 轴粗调步长 (m)，0.1mm')
    x_fine_ratio_arg = DeclareLaunchArgument(
        'x_fine_threshold_ratio', default_value='0.8',
        description='X 轴粗→细切换比例')
    x_coarse_wait_arg = DeclareLaunchArgument(
        'x_coarse_stabilize_cycles', default_value='10',
        description='X 轴粗调步间等待周期')
    x_fine_wait_arg = DeclareLaunchArgument(
        'x_fine_stabilize_cycles', default_value='50',
        description='X 轴精细步间等待周期')
    x_drift_arg = DeclareLaunchArgument(
        'x_drift_threshold', default_value='0.2',
        description='X 轴 HOLD 漂移阈值 (N)')
    x_sign_arg = DeclareLaunchArgument(
        'x_step_sign', default_value='1',
        description='X 轴运动方向符号: 1=正方向增加 |Fx|，-1=反方向')
    x_delay_arg = DeclareLaunchArgument(
        'x_start_delay', default_value='0.0',
        description='X 轴启动延迟 (s)，Z 进入 HOLD 后等待此秒数再启动 X')
    x_field_arg = DeclareLaunchArgument(
        'x_force_field', default_value='force.x',
        description='X 轴对应的 Wrench 字段')
    x_fsign_arg = DeclareLaunchArgument(
        'x_force_sign', default_value='1',
        description='X 轴力值符号翻转')

    # =========================================================================
    # Y 轴参数（切向力 Fy）
    # =========================================================================
    y_target_arg = DeclareLaunchArgument(
        'y_target_force', default_value='5.0',
        description='Y 轴目标力值 (N)')
    y_step_arg = DeclareLaunchArgument(
        'y_step_size', default_value='0.00005',
        description='Y 轴精细步长 (m)，0.05mm')
    y_approach_arg = DeclareLaunchArgument(
        'y_approach_step_size', default_value='0.0001',
        description='Y 轴粗调步长 (m)，0.1mm')
    y_fine_ratio_arg = DeclareLaunchArgument(
        'y_fine_threshold_ratio', default_value='0.8',
        description='Y 轴粗→细切换比例')
    y_coarse_wait_arg = DeclareLaunchArgument(
        'y_coarse_stabilize_cycles', default_value='10',
        description='Y 轴粗调步间等待周期')
    y_fine_wait_arg = DeclareLaunchArgument(
        'y_fine_stabilize_cycles', default_value='50',
        description='Y 轴精细步间等待周期')
    y_drift_arg = DeclareLaunchArgument(
        'y_drift_threshold', default_value='0.2',
        description='Y 轴 HOLD 漂移阈值 (N)')
    y_sign_arg = DeclareLaunchArgument(
        'y_step_sign', default_value='1',
        description='Y 轴运动方向符号: 1=正方向增加 |Fy|，-1=反方向')
    y_delay_arg = DeclareLaunchArgument(
        'y_start_delay', default_value='0.0',
        description='Y 轴启动延迟 (s)，Z 进入 HOLD 后等待此秒数再启动 Y')
    y_field_arg = DeclareLaunchArgument(
        'y_force_field', default_value='force.y',
        description='Y 轴对应的 Wrench 字段')
    y_fsign_arg = DeclareLaunchArgument(
        'y_force_sign', default_value='1',
        description='Y 轴力值符号翻转')

    # =========================================================================
    # 控制节点参数汇总
    # =========================================================================
    control_params = {
        'control_axes':       LaunchConfiguration('control_axes'),
        'start_x':            LaunchConfiguration('start_x'),
        'start_y':            LaunchConfiguration('start_y'),
        'start_z':            LaunchConfiguration('start_z'),
        'orientation_x':      LaunchConfiguration('orientation_x'),
        'orientation_y':      LaunchConfiguration('orientation_y'),
        'orientation_z':      LaunchConfiguration('orientation_z'),
        'orientation_w':      LaunchConfiguration('orientation_w'),
        'movej_speed':        LaunchConfiguration('movej_speed'),
        'hold_duration':      LaunchConfiguration('hold_duration'),
        'trajectory_duration': LaunchConfiguration('trajectory_duration'),
    }

    for ax in ('x', 'y', 'z'):
        for p in ('target_force', 'step_size', 'approach_step_size',
                  'fine_threshold_ratio', 'coarse_stabilize_cycles',
                  'fine_stabilize_cycles', 'drift_threshold',
                  'step_sign', 'start_delay', 'force_field', 'force_sign'):
            control_params[f'{ax}_{p}'] = LaunchConfiguration(f'{ax}_{p}')

    # =========================================================================
    # LaunchDescription
    # =========================================================================
    return LaunchDescription([
        control_axes_arg,
        start_x_arg, start_y_arg, start_z_arg,
        ori_x_arg, ori_y_arg, ori_z_arg, ori_w_arg,
        movej_speed_arg, hold_duration_arg, trajectory_duration_arg,
        # Z 轴
        z_target_arg, z_step_arg, z_approach_arg, z_fine_ratio_arg,
        z_coarse_wait_arg, z_fine_wait_arg, z_drift_arg, z_sign_arg,
        z_delay_arg, z_field_arg, z_fsign_arg,
        # X 轴
        x_target_arg, x_step_arg, x_approach_arg, x_fine_ratio_arg,
        x_coarse_wait_arg, x_fine_wait_arg, x_drift_arg, x_sign_arg,
        x_delay_arg, x_field_arg, x_fsign_arg,
        # Y 轴
        y_target_arg, y_step_arg, y_approach_arg, y_fine_ratio_arg,
        y_coarse_wait_arg, y_fine_wait_arg, y_drift_arg, y_sign_arg,
        y_delay_arg, y_field_arg, y_fsign_arg,
        # 节点
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
            parameters=[control_params],
        ),
    ])
