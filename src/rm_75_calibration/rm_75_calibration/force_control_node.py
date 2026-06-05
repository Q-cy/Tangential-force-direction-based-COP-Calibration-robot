#!/usr/bin/env python3
"""
恒力保压控制节点（三轴网格遍历模式）
- Z 轴：法向力 Fz 保压控制（步进试探）
- X 轴：切向力 Fy 保压控制（步进试探）
- Y 轴：切向力 Fx 保压控制（步进试探）
- 网格遍历：Z×Y×X 所有力目标组合依次执行
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String
from rm_ros_interfaces.msg import Movejp, Cartepos
import time
import sys
sys.path.insert(0, '/home/qcy/ros2_project_ws/src/rm_75_calibration/tang_7_12_Init_vaule_stable_COP_vec_cal_inter_realtime')


class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control_node')

        # ==================== Z 轴参数（法向力 Fz） ====================
        self.declare_parameter('z_target_force', 20.0)
        self.declare_parameter('z_step_size', 0.00005)
        self.declare_parameter('z_approach_step_size', 0.0001)
        self.declare_parameter('z_fine_threshold_ratio', 0.8)
        self.declare_parameter('z_coarse_stabilize_cycles', 10)
        self.declare_parameter('z_fine_stabilize_cycles', 50)
        self.declare_parameter('z_drift_threshold', 0.2)
        self.declare_parameter('z_force_tolerance', 0.1)

        # ==================== X 轴参数（切向力 Fy） ====================
        self.declare_parameter('x_target_force', 5.0)
        self.declare_parameter('x_step_size', 0.00005)
        self.declare_parameter('x_approach_step_size', 0.0001)
        self.declare_parameter('x_fine_threshold_ratio', 0.8)
        self.declare_parameter('x_coarse_stabilize_cycles', 10)
        self.declare_parameter('x_fine_stabilize_cycles', 50)
        self.declare_parameter('x_drift_threshold', 0.2)
        self.declare_parameter('x_step_sign', 1)
        self.declare_parameter('x_start_delay', 0.0)
        self.declare_parameter('x_force_tolerance', 0.1)

        # ==================== Y 轴参数（切向力 Fx） ====================
        self.declare_parameter('y_target_force', 2.0)
        self.declare_parameter('y_step_size', 0.00005)
        self.declare_parameter('y_approach_step_size', 0.0001)
        self.declare_parameter('y_fine_threshold_ratio', 0.8)
        self.declare_parameter('y_coarse_stabilize_cycles', 10)
        self.declare_parameter('y_fine_stabilize_cycles', 50)
        self.declare_parameter('y_drift_threshold', 0.2)
        self.declare_parameter('y_step_sign', 1)
        self.declare_parameter('y_start_delay', 0.0)
        self.declare_parameter('y_force_tolerance', 0.2)

        # ==================== 通用参数 ====================
        self.declare_parameter('movej_speed', 15)
        self.declare_parameter('start_x', 0.432)
        self.declare_parameter('start_y', -0.0065)
        self.declare_parameter('start_z', 0.18)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 1.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 0.0)
        self.declare_parameter('control_axes', 'xyz')
        self.declare_parameter('hold_duration', 10.0)

        # ==================== 网格参数 ====================
        self.declare_parameter('z_force_range_min', 5.0)
        self.declare_parameter('z_force_range_max', 20.0)
        self.declare_parameter('z_force_range_step', 5.0)
        self.declare_parameter('y_force_ratio', 0.2)
        self.declare_parameter('x_force_ratio', 0.2)
        self.declare_parameter('y_step_ratio', 0.2)
        self.declare_parameter('x_step_ratio', 0.2)
        self.declare_parameter('y_direction_mode', 3)
        self.declare_parameter('x_direction_mode', 3)
        self.declare_parameter('grid_hold_duration', 5.0)
        self.declare_parameter('grid_step_dwell', 0)
        self.declare_parameter('grid_unload_dwell', 0)
        self.declare_parameter('hold_only', True)
        self.declare_parameter('enable_cop_refine', True)

        self._read_params()

        # ========== 姿态 ==========
        self.ori = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]

        # ========== 订阅 ==========
        self.current_fz = 0.0
        self.current_fy = 0.0
        self.current_fx = 0.0
        self.force_sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._force_callback, 10
        )
        self.return_home_sub = self.create_subscription(
            String, '/force_control_return_home', self._return_home_callback, 10
        )

        # ========== 发布 ==========
        self.movej_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.cartepos_pub = self.create_publisher(Cartepos, '/rm_driver/movep_canfd_cmd', 10)
        self.valid_pub = self.create_publisher(String, '/force_control_state', 10)
        self.cop_trigger_pub = self.create_publisher(String, '/cop_trigger', 10)

        # ==================== Z 轴状态 ====================
        self.current_z = self.start_z
        self.z_phase = 'IDLE'
        self.z_wait_counter = 0
        self.z_prev_fz = 0.0
        self.z_prev_z = self.start_z
        self.z_error_before = 0.0
        self.z_hold_z = 0.0
        self.z_total_descent = 0.0
        self.z_hold_time = 0.0

        # ==================== X 轴状态 ====================
        self.current_x = self.start_x
        self.x_phase = 'IDLE'
        self.x_wait_counter = 0
        self.x_prev_fy = 0.0
        self.x_prev_x = self.start_x
        self.x_error_before = 0.0
        self.x_hold_x = 0.0

        # ==================== Y 轴状态 ====================
        self.current_y = self.start_y
        self.y_phase = 'IDLE'
        self.y_wait_counter = 0
        self.y_prev_f = 0.0
        self.y_prev_y = self.start_y
        self.y_error_before = 0.0
        self.y_hold_y = 0.0

        # ==================== 网格 ====================
        import numpy as np
        z_vals = list(np.arange(
            self.z_force_range_min,
            self.z_force_range_max + self.z_force_range_step * 0.5,
            self.z_force_range_step
        ))
        y_step = self.z_force_range_step * self.y_step_ratio
        x_step = self.z_force_range_step * self.x_step_ratio
        all_grid = []
        for z in z_vals:
            # Y: 只在 active_axes 中时生成目标值
            if 'y' in self.active_axes:
                y_max = z * self.y_force_ratio
                y_vals = []
                if self.y_direction_mode in (1, 3):  # 正
                    y_vals += list(np.arange(0, y_max + y_step * 0.5, y_step))
                if self.y_direction_mode in (2, 3):  # 负
                    y_vals += list(np.arange(-y_step, -y_max - y_step * 0.5, -y_step))
            else:
                y_vals = [0]
            # X: 只在 active_axes 中时生成目标值
            if 'x' in self.active_axes:
                x_max = z * self.x_force_ratio
                x_vals = []
                if self.x_direction_mode in (1, 3):  # 正
                    x_vals += list(np.arange(0, x_max + x_step * 0.5, x_step))
                if self.x_direction_mode in (2, 3):  # 负
                    x_vals += list(np.arange(-x_step, -x_max - x_step * 0.5, -x_step))
            else:
                x_vals = [0]
            for y in y_vals:
                for x in x_vals:
                    all_grid.append((z, y, x))
        self.grid_points = all_grid
        self.grid_index = 0
        self.grid_hold_start_time = None
        self.grid_dwell_start_time = None
        self.grid_phase = 'IDLE'  # IDLE → Z_CONTROL → XY_CONTROL → DWELL → RETURN → UNLOAD
        self.grid_dwell_counter = 0
        self.grid_unload_counter = 0
        self.z_dwell_counter = 0

        # ========== 全局状态 ==========
        self.state = 'MOVE_TO_START'

        # ========== 日志 ==========
        self._status_seq = 0

        # ========== 100Hz 控制定时器 ==========
        time.sleep(3.0)
        self.ctrl_timer = self.create_timer(0.01, self._control_callback)
        self.get_logger().info(
            f'恒力保压控制节点启动 | 控制轴={self.active_axes} | '
            f'网格点数={len(self.grid_points)}'
        )

    # ==================== 参数读取 ====================
    def _read_params(self):
        # Z
        self.z_target = self.get_parameter('z_target_force').value
        self.z_step = self.get_parameter('z_step_size').value
        self.z_approach_step = self.get_parameter('z_approach_step_size').value
        self.z_fine_ratio = self.get_parameter('z_fine_threshold_ratio').value
        self.z_coarse_wait = self.get_parameter('z_coarse_stabilize_cycles').value
        self.z_fine_wait = self.get_parameter('z_fine_stabilize_cycles').value
        self.z_drift = self.get_parameter('z_drift_threshold').value
        self.z_tolerance = self.get_parameter('z_force_tolerance').value
        self.z_force_limit = self.z_target * 1.5
        # X
        self.x_target = self.get_parameter('x_target_force').value
        self.x_step = self.get_parameter('x_step_size').value
        self.x_approach_step = self.get_parameter('x_approach_step_size').value
        self.x_fine_ratio = self.get_parameter('x_fine_threshold_ratio').value
        self.x_coarse_wait = self.get_parameter('x_coarse_stabilize_cycles').value
        self.x_fine_wait = self.get_parameter('x_fine_stabilize_cycles').value
        self.x_drift = self.get_parameter('x_drift_threshold').value
        self.x_step_sign = self.get_parameter('x_step_sign').value
        self.x_start_delay = self.get_parameter('x_start_delay').value
        self.x_tolerance = self.get_parameter('x_force_tolerance').value
        self.x_force_limit = self.x_target * 3.0
        # Y
        self.y_target = self.get_parameter('y_target_force').value
        self.y_step = self.get_parameter('y_step_size').value
        self.y_approach_step = self.get_parameter('y_approach_step_size').value
        self.y_fine_ratio = self.get_parameter('y_fine_threshold_ratio').value
        self.y_coarse_wait = self.get_parameter('y_coarse_stabilize_cycles').value
        self.y_fine_wait = self.get_parameter('y_fine_stabilize_cycles').value
        self.y_drift = self.get_parameter('y_drift_threshold').value
        self.y_step_sign = self.get_parameter('y_step_sign').value
        self.y_start_delay = self.get_parameter('y_start_delay').value
        self.y_tolerance = self.get_parameter('y_force_tolerance').value
        self.y_force_limit = self.y_target * 3.0
        # common
        self.movej_speed = self.get_parameter('movej_speed').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.ori_x = self.get_parameter('orientation_x').value
        self.ori_y = self.get_parameter('orientation_y').value
        self.ori_z = self.get_parameter('orientation_z').value
        self.ori_w = self.get_parameter('orientation_w').value
        self.hold_duration = self.get_parameter('hold_duration').value
        # control_axes
        self.control_axes = self.get_parameter('control_axes').value
        self.active_axes = set(self.control_axes.lower().replace(' ', ''))
        # grid
        self.z_force_range_min = self.get_parameter('z_force_range_min').value
        self.z_force_range_max = self.get_parameter('z_force_range_max').value
        self.z_force_range_step = self.get_parameter('z_force_range_step').value
        self.y_force_ratio = self.get_parameter('y_force_ratio').value
        self.x_force_ratio = self.get_parameter('x_force_ratio').value
        self.y_step_ratio = self.get_parameter('y_step_ratio').value
        self.x_step_ratio = self.get_parameter('x_step_ratio').value
        self.y_direction_mode = self.get_parameter('y_direction_mode').value
        self.x_direction_mode = self.get_parameter('x_direction_mode').value
        self.grid_hold_duration = self.get_parameter('grid_hold_duration').value
        self.grid_step_dwell = self.get_parameter('grid_step_dwell').value
        self.grid_unload_dwell = self.get_parameter('grid_unload_dwell').value
        self.hold_only = self.get_parameter('hold_only').value
        self.enable_cop_refine = self.get_parameter('enable_cop_refine').value

    # ==================== 回初始位置回调 ====================
    def _return_home_callback(self, msg):
        self.get_logger().info('收到回初始位置指令')
        self.state = 'MOVE_TO_START'

    # ==================== 力传感器回调 ====================
    def _force_callback(self, msg):
        self.current_fz = msg.wrench.force.z * (-1)
        self.current_fy = msg.wrench.force.y
        self.current_fx = msg.wrench.force.x

    # ==================== 100Hz 主循环 ====================
    def _control_callback(self):
        if self.state == 'MOVE_TO_START':
            self._enter_move_to_start()
            return
        if self.state == 'WAIT_MOVEJ':
            self._check_movej_done()
            return
        if self.state == 'ESTOP':
            return
        if self.state == 'DONE':
            return

        # ===== 发布 valid 状态（DWELL且全HOLD时为网格点序号，其他为0） =====
        valid = self.grid_index + 1 if self.grid_phase == 'DWELL' and self._all_axes_hold() else 0
        self.valid_pub.publish(String(data=str(valid)))

        # ===== 安全检查 =====
        if abs(self.current_fz) > self.z_force_range_max * 1.5:
            self.get_logger().error(f'Z力={self.current_fz:.1f}N 超过安全限 {self.z_force_range_max*1.5}N → ESTOP')
            self.state = 'ESTOP'
            return
        if 'z' in self.active_axes and abs(self.current_fz) > self.z_force_limit:
            self.get_logger().warn(f'Z力超限 Fz={self.current_fz:.2f}>{self.z_force_limit}N')
        if 'x' in self.active_axes and abs(self.current_fx) > self.x_force_limit:
            self.get_logger().warn(f'X力超限 Fx={self.current_fx:.2f}>{self.x_force_limit}N')
        if 'y' in self.active_axes and abs(self.current_fy) > self.y_force_limit:
            self.get_logger().warn(f'Y力超限 Fy={self.current_fy:.2f}>{self.y_force_limit}N')

        # ===== 周期打印力值和坐标（每秒一次，覆盖上一次） =====
        self._status_seq += 1
        if self._status_seq % 100 == 0:
            lines = [
                f'[S] {self.grid_index+1}/{len(self.grid_points)} {self.grid_phase}',
                f'目标 Fz={self.z_target:.1f} Fy={self.y_target:.1f} Fx={self.x_target:.1f}',
                f'实际 Fz={self.current_fz:.1f} Fy={self.current_fy:.1f} Fx={self.current_fx:.1f}',
                f'坐标 Z={self.current_z:.4f} Y={self.current_y:.4f} X={self.current_x:.4f}',
                f'阶段 {self.z_phase} {self.y_phase} {self.x_phase}',
            ]
            # 上移 5 行清除上次输出，再打印新内容
            if self._status_seq > 100:
                sys.stdout.write('\033[5A\033[J')
            sys.stdout.write('\n'.join(lines) + '\n')
            sys.stdout.flush()

        # ===== 网格阶段状态机 =====
        if self.grid_phase == 'Z_CONTROL':
            any_publish = False
            any_publish |= self._run_z_machine()

            # 保压计数
            if self.hold_only:
                # Z 到 HOLD 就进 XY_CONTROL
                if self.z_phase == 'HOLD':
                    self._start_xy_control()
            else:
                # 全部轴 HOLD 后才开始计数
                if self._all_axes_hold():
                    if self.z_dwell_counter > 0:
                        self.z_dwell_counter -= 1
                    if self.z_dwell_counter == 0:
                        self._start_xy_control()
                else:
                    self.z_dwell_counter = self.grid_step_dwell

            # X/Y 维持 0N
            any_publish |= self._run_x_machine()
            any_publish |= self._run_y_machine()

            if any_publish:
                self._publish_cartepos()

        elif self.grid_phase == 'XY_CONTROL':
            any_publish = False
            any_publish |= self._run_z_machine()
            any_publish |= self._run_x_machine()
            any_publish |= self._run_y_machine()

            # 到位 → DWELL
            if self.hold_only:
                # X/Y 各自到 HOLD 就进 DWELL
                if self.x_phase == 'HOLD' and self.y_phase == 'HOLD':
                    self._start_dwell()
            else:
                # 全部 HOLD 才进 DWELL
                if self._all_axes_hold():
                    self._start_dwell()
            if any_publish:
                self._publish_cartepos()

        elif self.grid_phase == 'DWELL':
            # 保压阶段：继续力控（漂移纠偏）
            any_publish = False
            any_publish |= self._run_z_machine()
            any_publish |= self._run_x_machine()
            any_publish |= self._run_y_machine()
            # 只有所有轴都 HOLD 时才计数
            if self._all_axes_hold():
                self.grid_dwell_counter -= 1
            if self.grid_dwell_counter <= 0:
                self._start_return()
            if any_publish:
                self._publish_cartepos()

        elif self.grid_phase == 'RETURN':
            # 只等 MoveJp 完成，不发布 Cartepos 避免位置跳变
            if time.time() >= self._move_deadline:
                self._start_unload()

        elif self.grid_phase == 'UNLOAD':
            # MoveJp 已完成，等待卸载时间
            self.grid_unload_counter -= 1
            if self.grid_unload_counter <= 0:
                self._advance_grid()

    # ==================== 网格阶段辅助方法 ====================
    def _all_axes_hold(self):
        return all(self._get_axis_phase(a) == 'HOLD' for a in self.active_axes)

    def _start_xy_control(self):
        """Z 保压完成，更新 X/Y 目标并开始力控"""
        if self.enable_cop_refine:
            self.cop_trigger_pub.publish(String(data='refine'))
        self.get_logger().info(f'[GRID] XY_CONTROL entry')
        self.y_target = self._grid_y_target
        self.y_force_limit = max(abs(self._grid_y_target) * 3.0, 5.0)
        self._reset_axis('y')
        self.x_target = self._grid_x_target
        self.x_force_limit = max(abs(self._grid_x_target) * 3.0, 5.0)
        self._reset_axis('x')
        self.grid_phase = 'XY_CONTROL'
        self.get_logger().info(
            f'[GRID] XY_CONTROL y={self._grid_y_target:.1f} x={self._grid_x_target:.1f} '
            f'({self.grid_index+1}/{len(self.grid_points)})'
        )

    def _start_dwell(self):
        # 保压时间：优先用 grid_step_dwell（循环数），否则用 grid_hold_duration（秒）
        if self.grid_step_dwell > 0:
            self.grid_dwell_counter = self.grid_step_dwell
        else:
            self.grid_dwell_counter = int(self.grid_hold_duration * 100)
        self.grid_phase = 'DWELL'
        self.get_logger().info(
            f'[GRID] DWELL {self.grid_dwell_counter} ({self.grid_index+1}/{len(self.grid_points)})'
        )

    def _start_return(self):
        self.get_logger().info('[GRID] DWELL完成 → RETURN 回初始位置')
        self.grid_phase = 'RETURN'
        msg = Movejp()
        msg.pose.position.x = float(self.start_x)
        msg.pose.position.y = float(self.start_y)
        msg.pose.position.z = float(self.start_z)
        msg.pose.orientation.x = float(self.ori[0])
        msg.pose.orientation.y = float(self.ori[1])
        msg.pose.orientation.z = float(self.ori[2])
        msg.pose.orientation.w = float(self.ori[3])
        msg.speed = self.movej_speed
        msg.trajectory_connect = 0
        msg.block = False
        self.movej_pub.publish(msg)
        self._move_deadline = time.time() + 5.0

    def _start_unload(self):
        self.get_logger().info('[GRID] 到达初始位置 → UNLOAD')
        self.grid_unload_counter = self.grid_unload_dwell
        self.grid_phase = 'UNLOAD'

    def _advance_grid(self):
        self.grid_index += 1
        if self.grid_index >= len(self.grid_points):
            self.get_logger().info('[GRID] 全部网格点完成')
            self.state = 'DONE'
            return

        # 重置坐标为初始值，确保每个网格点从同一起点开始
        self.current_x = self.start_x
        self.current_y = self.start_y
        self.current_z = self.start_z

        z_t, y_t, x_t = self.grid_points[self.grid_index]
        self.get_logger().info(
            f'[GRID] 移动到点 {self.grid_index + 1}/{len(self.grid_points)}: '
            f'z={z_t}, y={y_t}, x={x_t}'
        )

        # 保存网格点的实际目标
        self._grid_y_target = y_t if y_t is not None else 0
        self._grid_x_target = x_t if x_t is not None else 0

        if z_t is not None:
            self.z_target = z_t
            self.z_force_limit = max(z_t * 1.5, 5.0)
            self._reset_axis('z')
        # X/Y 初始目标为 0，Z 保压后再更新（不活跃的轴直接 HOLD）
        if 'y' in self.active_axes:
            self.y_target = 0
            self.y_force_limit = 5.0
            self._reset_axis('y')
        else:
            self.y_target = 0
            self.y_phase = 'HOLD'
        if 'x' in self.active_axes:
            self.x_target = 0
            self.x_force_limit = 5.0
            self._reset_axis('x')
        else:
            self.x_target = 0
            self.x_phase = 'HOLD'

        self.grid_hold_start_time = None
        self.grid_dwell_start_time = None
        self.grid_phase = 'Z_CONTROL'
        self._cop_triggered_this_point = False

    def _get_axis_phase(self, axis):
        if axis == 'z': return self.z_phase
        if axis == 'y': return self.y_phase
        if axis == 'x': return self.x_phase

    def _reset_axis(self, axis):
        if axis == 'z':
            if self.z_target == 0:
                self.z_phase = 'HOLD'
                self.z_dwell_counter = self.grid_step_dwell
            else:
                self.z_phase = 'APPROACH'
                self.z_wait_counter = 0
                self.z_prev_fz = self.current_fz
                self.z_prev_z = self.current_z
                self.z_dwell_counter = self.grid_step_dwell
        elif axis == 'y':
            self.y_phase = 'APPROACH'
            self.y_wait_counter = 0
            self.y_prev_f = abs(self.current_fy)
            self.y_prev_y = self.current_y
        elif axis == 'x':
            self.x_phase = 'APPROACH'
            self.x_wait_counter = 0
            self.x_prev_fy = abs(self.current_fx)
            self.x_prev_x = self.current_x

    # ==================== Z 轴状态机 ====================
    def _run_z_machine(self):
        if self.z_phase == 'HOLD':
            return self._z_hold_monitor()
        if self.z_wait_counter > 0:
            self.z_wait_counter -= 1
            return True
        if self.z_phase == 'APPROACH':
            self._z_evaluate_approach()
        elif self.z_phase == 'FINE_TUNE_WAIT':
            self._z_evaluate_fine_tune()
        elif self.z_phase == 'RECOVER_WAIT':
            self._z_evaluate_recover()
        return True

    def _z_hold_monitor(self):
        error = abs(self.z_target - self.current_fz)
        if error > self.z_drift:
            self.z_prev_fz = self.current_fz
            self.z_prev_z = self.current_z
            self.z_error_before = error
            direction = 'down' if self.current_fz < self.z_target else 'up'
            self._z_do_step(self.z_step, self.z_fine_wait, direction=direction)
            self.z_phase = 'RECOVER_WAIT'
            return True
        return False

    def _z_evaluate_approach(self):
        if self.current_fz >= self.z_target:
            e_prev = abs(self.z_target - self.z_prev_fz)
            e_curr = abs(self.z_target - self.current_fz)
            if e_prev <= e_curr:
                self.current_z = self.z_prev_z
                self.z_phase = 'HOLD'
                self.z_hold_time = time.time()
                self._publish_cartepos()
            else:
                self.z_error_before = e_curr
                self.z_hold_z = self.current_z
                self._z_do_step(self.z_step, self.z_fine_wait)
                self.z_phase = 'FINE_TUNE_WAIT'
            return

        self.z_prev_fz = self.current_fz
        self.z_prev_z = self.current_z
        fine_th = self.z_target * self.z_fine_ratio
        if self.current_fz < fine_th:
            self._z_do_step(self.z_approach_step, self.z_coarse_wait)
        else:
            self._z_do_step(self.z_step, self.z_fine_wait)

    def _z_evaluate_fine_tune(self):
        e_after = abs(self.z_target - self.current_fz)
        if e_after > self.z_error_before:
            self.current_z = self.z_hold_z
            self._publish_cartepos()
        self.z_phase = 'HOLD'
        self.z_hold_time = time.time()

    def _z_evaluate_recover(self):
        e_after = abs(self.z_target - self.current_fz)
        if e_after > self.z_error_before:
            self.current_z = self.z_prev_z
            self._publish_cartepos()
        self.z_phase = 'HOLD'

    def _z_do_step(self, step, wait_cycles, direction='down'):
        if direction == 'down':
            self.current_z -= step
            self.z_total_descent += step
        else:
            self.current_z += step
        self.z_wait_counter = wait_cycles

    # ==================== X 轴状态机 ====================
    def _run_x_machine(self):
        if self.x_phase == 'IDLE':
            if self.x_target == 0:
                self.x_phase = 'HOLD'
                return True
            y_ready = (self.y_phase == 'HOLD') or ('y' not in self.active_axes)
            if self.z_phase == 'HOLD' and y_ready and self.z_hold_time > 0:
                if time.time() - self.z_hold_time >= self.x_start_delay:
                    self.x_phase = 'APPROACH'
                    self.x_wait_counter = 0
                    self.x_prev_fy = abs(self.current_fx)
                    self.x_prev_x = self.current_x
                else:
                    return False
            else:
                return False

        if self.x_phase == 'HOLD':
            return self._x_hold_monitor()

        if self.x_wait_counter > 0:
            self.x_wait_counter -= 1
            return True

        if self.x_phase == 'APPROACH':
            self._x_evaluate_approach()
        elif self.x_phase == 'FINE_TUNE_WAIT':
            self._x_evaluate_fine_tune()
        elif self.x_phase == 'RECOVER_WAIT':
            self._x_evaluate_recover()
        return True

    def _x_hold_monitor(self):
        error = abs(abs(self.x_target) - abs(self.current_fx))
        if error > self.x_drift:
            self.x_prev_fy = abs(self.current_fx)
            self.x_prev_x = self.current_x
            self.x_error_before = error
            if self.x_target == 0:
                direction = 'fwd' if self.current_fx < 0 else 'rev'
            else:
                if self.x_target >= 0:
                    direction = 'fwd' if self.current_fx < self.x_target else 'rev'
                else:
                    direction = 'rev' if self.current_fx < self.x_target else 'fwd'
            self._x_do_step(self.x_step, self.x_fine_wait, direction=direction)
            self.x_phase = 'RECOVER_WAIT'
            return True
        return False

    def _x_evaluate_approach(self):
        fx_abs = abs(self.current_fx)
        x_target_abs = abs(self.x_target)
        # target=0 特殊处理：直接根据力方向步进
        if self.x_target == 0:
            if fx_abs <= self.x_tolerance:
                self.x_phase = 'HOLD'
            elif self.current_fx > 0:
                self._x_do_step(self.x_step, self.x_fine_wait, direction='rev')
            else:
                self._x_do_step(self.x_step, self.x_fine_wait, direction='fwd')
            return
        # 跨越判断：正目标 force>=target，负目标 force<=target
        crossed = (self.x_target >= 0 and self.current_fx >= self.x_target) or \
                  (self.x_target < 0 and self.current_fx <= self.x_target)
        if crossed:
            e_prev = abs(x_target_abs - self.x_prev_fy)
            e_curr = abs(x_target_abs - fx_abs)
            if e_prev <= e_curr:
                self.current_x = self.x_prev_x
                self.x_phase = 'HOLD'
                self._publish_cartepos()
            else:
                self.x_error_before = e_curr
                self.x_hold_x = self.current_x
                step_dir = 'fwd' if self.x_target >= 0 else 'rev'
                self._x_do_step(self.x_step, self.x_fine_wait, direction=step_dir)
                self.x_phase = 'FINE_TUNE_WAIT'
            return

        self.x_prev_fy = fx_abs
        self.x_prev_x = self.current_x
        fine_th = x_target_abs * self.x_fine_ratio
        step_dir = 'fwd' if self.x_target >= 0 else 'rev'
        if fx_abs < fine_th:
            self._x_do_step(self.x_approach_step, self.x_coarse_wait, direction=step_dir)
        else:
            self._x_do_step(self.x_step, self.x_fine_wait, direction=step_dir)

    def _x_evaluate_fine_tune(self):
        e_after = abs(abs(self.x_target) - abs(self.current_fx))
        if e_after > self.x_error_before:
            self.current_x = self.x_hold_x
            self._publish_cartepos()
        self.x_phase = 'HOLD'

    def _x_evaluate_recover(self):
        e_after = abs(abs(self.x_target) - abs(self.current_fx))
        if e_after > self.x_error_before:
            self.current_x = self.x_prev_x
            self._publish_cartepos()
        self.x_phase = 'HOLD'

    def _x_do_step(self, step, wait_cycles, direction='fwd'):
        if direction == 'fwd':
            self.current_x += self.x_step_sign * step
        else:
            self.current_x -= self.x_step_sign * step
        self.x_wait_counter = wait_cycles

    # ==================== Y 轴状态机 ====================
    def _run_y_machine(self):
        if self.y_phase == 'IDLE':
            if self.y_target == 0:
                self.y_phase = 'HOLD'
                return True
            if self.z_phase == 'HOLD' and self.z_hold_time > 0:
                if time.time() - self.z_hold_time >= self.y_start_delay:
                    self.y_phase = 'APPROACH'
                    self.y_wait_counter = 0
                    self.y_prev_f = abs(self.current_fy)
                    self.y_prev_y = self.current_y
                else:
                    return False
            else:
                return False

        if self.y_phase == 'HOLD':
            return self._y_hold_monitor()

        if self.y_wait_counter > 0:
            self.y_wait_counter -= 1
            return True

        if self.y_phase == 'APPROACH':
            self._y_evaluate_approach()
        elif self.y_phase == 'FINE_TUNE_WAIT':
            self._y_evaluate_fine_tune()
        elif self.y_phase == 'RECOVER_WAIT':
            self._y_evaluate_recover()
        return True

    def _y_hold_monitor(self):
        error = abs(abs(self.y_target) - abs(self.current_fy))
        if error > self.y_drift:
            self.y_prev_f = abs(self.current_fy)
            self.y_prev_y = self.current_y
            self.y_error_before = error
            if self.y_target == 0:
                direction = 'fwd' if self.current_fy < 0 else 'rev'
            else:
                if self.y_target >= 0:
                    direction = 'fwd' if self.current_fy < self.y_target else 'rev'
                else:
                    direction = 'rev' if self.current_fy < self.y_target else 'fwd'
            self._y_do_step(self.y_step, self.y_fine_wait, direction=direction)
            self.y_phase = 'RECOVER_WAIT'
            return True
        return False

    def _y_evaluate_approach(self):
        fy_abs = abs(self.current_fy)
        y_target_abs = abs(self.y_target)
        # target=0 特殊处理：直接根据力方向步进
        if self.y_target == 0:
            if fy_abs <= self.y_tolerance:
                self.y_phase = 'HOLD'
            elif self.current_fy > 0:
                self._y_do_step(self.y_step, self.y_fine_wait, direction='rev')
            else:
                self._y_do_step(self.y_step, self.y_fine_wait, direction='fwd')
            return
        # 跨越判断：正目标 force>=target，负目标 force<=target
        crossed = (self.y_target >= 0 and self.current_fy >= self.y_target) or \
                  (self.y_target < 0 and self.current_fy <= self.y_target)
        if crossed:
            e_prev = abs(y_target_abs - self.y_prev_f)
            e_curr = abs(y_target_abs - fy_abs)
            if e_prev <= e_curr:
                self.current_y = self.y_prev_y
                self.y_phase = 'HOLD'
                self._publish_cartepos()
            else:
                self.y_error_before = e_curr
                self.y_hold_y = self.current_y
                step_dir = 'fwd' if self.y_target >= 0 else 'rev'
                self._y_do_step(self.y_step, self.y_fine_wait, direction=step_dir)
                self.y_phase = 'FINE_TUNE_WAIT'
            return

        self.y_prev_f = fy_abs
        self.y_prev_y = self.current_y
        fine_th = y_target_abs * self.y_fine_ratio
        step_dir = 'fwd' if self.y_target >= 0 else 'rev'
        if fy_abs < fine_th:
            self._y_do_step(self.y_approach_step, self.y_coarse_wait, direction=step_dir)
        else:
            self._y_do_step(self.y_step, self.y_fine_wait, direction=step_dir)

    def _y_evaluate_fine_tune(self):
        e_after = abs(abs(self.y_target) - abs(self.current_fy))
        if e_after > self.y_error_before:
            self.current_y = self.y_hold_y
            self._publish_cartepos()
        self.y_phase = 'HOLD'

    def _y_evaluate_recover(self):
        e_after = abs(abs(self.y_target) - abs(self.current_fy))
        if e_after > self.y_error_before:
            self.current_y = self.y_prev_y
            self._publish_cartepos()
        self.y_phase = 'HOLD'

    def _y_do_step(self, step, wait_cycles, direction='fwd'):
        if direction == 'fwd':
            self.current_y += self.y_step_sign * step
        else:
            self.current_y -= self.y_step_sign * step
        self.y_wait_counter = wait_cycles

    # ==================== 初始定位 ====================
    def _enter_move_to_start(self):
        self.get_logger().info(f'MoveJp → ({self.start_x:.4f}, {self.start_y:.4f}, {self.start_z:.4f})')
        msg = Movejp()
        msg.pose.position.x = float(self.start_x)
        msg.pose.position.y = float(self.start_y)
        msg.pose.position.z = float(self.start_z)
        msg.pose.orientation.x = float(self.ori[0])
        msg.pose.orientation.y = float(self.ori[1])
        msg.pose.orientation.z = float(self.ori[2])
        msg.pose.orientation.w = float(self.ori[3])
        msg.speed = self.movej_speed
        msg.trajectory_connect = 0
        msg.block = False
        self.movej_pub.publish(msg)
        self.current_z = self.start_z
        self.current_x = self.start_x
        self.current_y = self.start_y
        self._move_deadline = time.time() + 5.0
        self.state = 'WAIT_MOVEJ'

    def _check_movej_done(self):
        if time.time() >= self._move_deadline:
            # 应用首个网格点
            z_t, y_t, x_t = self.grid_points[0]
            if z_t is not None:
                self.z_target = z_t
                self.z_force_limit = max(z_t * 1.5, 5.0)

            self.get_logger().info(
                f'到达初始位姿 → 网格点 1/{len(self.grid_points)}: '
                f'z={z_t}, y={y_t}, x={x_t}'
            )

            # 保存实际网格目标
            self._grid_y_target = y_t if y_t is not None else 0
            self._grid_x_target = x_t if x_t is not None else 0

            if 'z' in self.active_axes:
                self.z_phase = 'APPROACH'
                self.z_wait_counter = 0
                self.z_total_descent = 0.0
                self.z_prev_fz = self.current_fz
                self.z_prev_z = self.current_z
                self.z_dwell_counter = self.grid_step_dwell

            # X/Y 初始目标为 0，Z 保压后再更新（不活跃的轴直接 HOLD）
            if 'y' in self.active_axes:
                self.y_target = 0
                self.y_force_limit = 5.0
                self.y_phase = 'APPROACH'
                self.y_prev_f = abs(self.current_fy)
                self.y_prev_y = self.current_y
            else:
                self.y_target = 0
                self.y_phase = 'HOLD'
            if 'x' in self.active_axes:
                self.x_target = 0
                self.x_force_limit = 5.0
                self.x_phase = 'APPROACH'
                self.x_prev_fy = abs(self.current_fx)
                self.x_prev_x = self.current_x
            else:
                self.x_target = 0
                self.x_phase = 'HOLD'

            self.state = 'FORCE_CONTROL'
            self.grid_phase = 'Z_CONTROL'
            self._cop_triggered_this_point = False

    # ==================== 发布 Cartepos ====================
    def _publish_cartepos(self):
        msg = Cartepos()
        msg.pose.position.x = float(self.current_x)
        msg.pose.position.y = float(self.current_y)
        msg.pose.position.z = float(self.current_z)
        msg.pose.orientation.x = float(self.ori[0])
        msg.pose.orientation.y = float(self.ori[1])
        msg.pose.orientation.z = float(self.ori[2])
        msg.pose.orientation.w = float(self.ori[3])
        msg.follow = True
        self.cartepos_pub.publish(msg)

    # ==================== 清理 ====================
    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()