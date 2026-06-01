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
import itertools


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
        from rclpy.parameter import Parameter
        self.declare_parameter('z_target_force_grid', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('y_target_force_grid', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('x_target_force_grid', Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter('grid_hold_duration', 5.0)
        self.declare_parameter('grid_step_dwell', 0)
        self.declare_parameter('grid_unload_dwell', 0)

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
        z_vals = self.z_target_grid if self.z_target_grid else [self.z_target]
        y_vals = self.y_target_grid if self.y_target_grid else [self.y_target]
        x_vals = self.x_target_grid if self.x_target_grid else [self.x_target]

        if 'z' not in self.active_axes:
            z_vals = [None]
        if 'y' not in self.active_axes:
            y_vals = [None]
        if 'x' not in self.active_axes:
            x_vals = [None]

        self.grid_points = list(itertools.product(z_vals, y_vals, x_vals))
        self.grid_index = 0
        self.grid_hold_start_time = None
        self.grid_dwell_start_time = None
        self.grid_phase = 'IDLE'  # IDLE → FORCE_CONTROL → DWELL → RETURN → UNLOAD
        self.grid_dwell_counter = 0
        self.grid_unload_counter = 0

        # ========== 全局状态 ==========
        self.state = 'MOVE_TO_START'

        # ========== 日志 ==========
        self._log_seq = 0
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
        self.z_target_grid = self.get_parameter('z_target_force_grid').value
        self.y_target_grid = self.get_parameter('y_target_force_grid').value
        self.x_target_grid = self.get_parameter('x_target_force_grid').value
        self.grid_hold_duration = self.get_parameter('grid_hold_duration').value
        self.grid_step_dwell = self.get_parameter('grid_step_dwell').value
        self.grid_unload_dwell = self.get_parameter('grid_unload_dwell').value

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

        # ===== 安全检查 =====
        if 'z' in self.active_axes and abs(self.current_fz) > self.z_force_limit:
            self.get_logger().error(f'Z力超限 Fz={self.current_fz:.2f}>{self.z_force_limit}N')
            self.state = 'ESTOP'; return
        if 'x' in self.active_axes and abs(self.current_fx) > self.x_force_limit:
            self.get_logger().error(f'X力超限 Fx={self.current_fx:.2f}>{self.x_force_limit}N')
            self.state = 'ESTOP'; return
        if 'y' in self.active_axes and abs(self.current_fy) > self.y_force_limit:
            self.get_logger().error(f'Y力超限 Fy={self.current_fy:.2f}>{self.y_force_limit}N')
            self.state = 'ESTOP'; return

        # ===== 周期打印力值和坐标（每秒一次） =====
        self._status_seq += 1
        if self._status_seq % 100 == 0:
            self.get_logger().info(
                f'[STATUS] 网格{self.grid_index+1}/{len(self.grid_points)} phase={self.grid_phase} | '
                f'力 Fz={self.current_fz:.2f} Fy={self.current_fy:.2f} Fx={self.current_fx:.2f}N | '
                f'坐标 X={self.current_x:.6f} Y={self.current_y:.6f} Z={self.current_z:.6f} | '
                f'Z:{self.z_phase} X:{self.x_phase} Y:{self.y_phase}'
            )

        # ===== 网格阶段状态机 =====
        if self.grid_phase == 'FORCE_CONTROL':
            # 运行各轴状态机
            any_publish = False
            if 'z' in self.active_axes:
                any_publish |= self._run_z_machine()
            if 'x' in self.active_axes:
                any_publish |= self._run_x_machine()
            if 'y' in self.active_axes:
                any_publish |= self._run_y_machine()
            # 检查是否全部到位
            if self._all_axes_hold():
                self._start_dwell()
            if any_publish:
                self._publish_cartepos()

        elif self.grid_phase == 'DWELL':
            # 保压阶段：继续力控（漂移纠偏）
            any_publish = False
            if 'z' in self.active_axes:
                any_publish |= self._run_z_machine()
            if 'x' in self.active_axes:
                any_publish |= self._run_x_machine()
            if 'y' in self.active_axes:
                any_publish |= self._run_y_machine()
            self.grid_dwell_counter -= 1
            if self.grid_dwell_counter <= 0:
                self._start_return()
            if any_publish:
                self._publish_cartepos()

        elif self.grid_phase == 'RETURN':
            # 只等 MoveJp 完成，不发布 Cartepos（避免冲突）
            if time.time() >= self._move_deadline:
                self._start_unload()

        elif self.grid_phase == 'UNLOAD':
            # MoveJp 已完成，发布 Cartepos 维持初始位置
            self._publish_cartepos()
            self.grid_unload_counter -= 1
            if self.grid_unload_counter <= 0:
                self._advance_grid()

    # ==================== 网格阶段辅助方法 ====================
    def _all_axes_hold(self):
        return all(self._get_axis_phase(a) == 'HOLD' for a in self.active_axes)

    def _start_dwell(self):
        # 保压时间：优先用 grid_step_dwell（循环数），否则用 grid_hold_duration（秒）
        if self.grid_step_dwell > 0:
            self.grid_dwell_counter = self.grid_step_dwell
        else:
            self.grid_dwell_counter = int(self.grid_hold_duration * 100)
        self.grid_phase = 'DWELL'
        self.get_logger().info(
            f'[GRID] 全部轴HOLD → DWELL {self.grid_dwell_counter} 周期 '
            f'(点 {self.grid_index + 1}/{len(self.grid_points)})'
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
        # 重置位置
        self.current_z = self.start_z
        self.current_x = self.start_x
        self.current_y = self.start_y
        self._publish_cartepos()
        self.grid_unload_counter = self.grid_unload_dwell
        self.grid_phase = 'UNLOAD'

    def _advance_grid(self):
        self.grid_index += 1
        if self.grid_index >= len(self.grid_points):
            self.get_logger().info('[GRID] 全部网格点完成')
            self.state = 'DONE'
            return

        z_t, y_t, x_t = self.grid_points[self.grid_index]
        self.get_logger().info(
            f'[GRID] 移动到点 {self.grid_index + 1}/{len(self.grid_points)}: '
            f'z={z_t}, y={y_t}, x={x_t}'
        )

        if z_t is not None:
            self.z_target = z_t
            self.z_force_limit = max(z_t * 1.5, 5.0)
            self._reset_axis('z')
        if y_t is not None:
            self.y_target = y_t
            self.y_force_limit = max(y_t * 3.0, 5.0)
            self._reset_axis('y')
        if x_t is not None:
            self.x_target = x_t
            self.x_force_limit = max(x_t * 3.0, 5.0)
            self._reset_axis('x')

        self.grid_hold_start_time = None
        self.grid_dwell_start_time = None
        self.grid_phase = 'FORCE_CONTROL'

    def _get_axis_phase(self, axis):
        if axis == 'z': return self.z_phase
        if axis == 'y': return self.y_phase
        if axis == 'x': return self.x_phase

    def _reset_axis(self, axis):
        if axis == 'z':
            if self.z_target == 0:
                self.z_phase = 'HOLD'
            else:
                self.z_phase = 'APPROACH'
                self.z_wait_counter = 0
                self.z_prev_fz = self.current_fz
                self.z_prev_z = self.current_z
        elif axis == 'y':
            if self.y_target == 0:
                self.y_phase = 'HOLD'
            else:
                self.y_phase = 'IDLE'  # 等 Z HOLD 后再启动
        elif axis == 'x':
            if self.x_target == 0:
                self.x_phase = 'HOLD'
            else:
                self.x_phase = 'IDLE'  # 等 Y HOLD 后再启动

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
            self.get_logger().info(
                f'[Z-HOLD] 漂移 Fz={self.current_fz:.2f}N err={error:.2f}>{self.z_drift}N → 向{direction}步进'
            )
            self._z_do_step(self.z_step, self.z_fine_wait, direction=direction)
            self.z_phase = 'RECOVER_WAIT'
            return True
        return False

    def _z_evaluate_approach(self):
        if self.current_fz >= self.z_target:
            e_prev = abs(self.z_target - self.z_prev_fz)
            e_curr = abs(self.z_target - self.current_fz)
            self.get_logger().info(
                f'[Z] 跨越 {self.z_target:.1f}N | '
                f'prev Fz={self.z_prev_fz:.2f} err={e_prev:.2f} | curr Fz={self.current_fz:.2f} err={e_curr:.2f}'
            )
            if e_prev <= e_curr:
                self.current_z = self.z_prev_z
                self.get_logger().info(f'[Z] >>> prev更接近 err={e_prev:.2f}≤{e_curr:.2f} → HOLD Z={self.current_z:.6f}')
                self.z_phase = 'HOLD'
                self.z_hold_time = time.time()
                self._publish_cartepos()
            else:
                self.z_error_before = e_curr
                self.z_hold_z = self.current_z
                self.get_logger().info(f'[Z] curr更接近 err={e_curr:.2f}<{e_prev:.2f} → 精调一步')
                self._z_do_step(self.z_step, self.z_fine_wait)
                self.z_phase = 'FINE_TUNE_WAIT'
            return

        self.z_prev_fz = self.current_fz
        self.z_prev_z = self.current_z
        fine_th = self.z_target * self.z_fine_ratio
        if self.current_fz < fine_th:
            self._z_do_step(self.z_approach_step, self.z_coarse_wait, tag='Z粗')
        else:
            self._z_do_step(self.z_step, self.z_fine_wait, tag='Z细')

    def _z_evaluate_fine_tune(self):
        e_after = abs(self.z_target - self.current_fz)
        self.get_logger().info(f'[Z] 精调 err {self.z_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.z_error_before:
            self.get_logger().info(f'[Z] >>> 精调OK → HOLD Z={self.current_z:.6f}')
        else:
            self.current_z = self.z_hold_z
            self.get_logger().info(f'[Z] >>> 回退 → HOLD Z={self.current_z:.6f}')
            self._publish_cartepos()
        self.z_phase = 'HOLD'
        self.z_hold_time = time.time()

    def _z_evaluate_recover(self):
        e_after = abs(self.z_target - self.current_fz)
        self.get_logger().info(f'[Z] 纠偏 err {self.z_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.z_error_before:
            self.get_logger().info(f'[Z] >>> 纠偏OK → HOLD')
        else:
            self.current_z = self.z_prev_z
            self.get_logger().info(f'[Z] >>> 纠偏无效 回退 → HOLD Z={self.current_z:.6f}')
            self._publish_cartepos()
        self.z_phase = 'HOLD'

    def _z_do_step(self, step, wait_cycles, direction='down', tag=''):
        if direction == 'down':
            self.current_z -= step
            self.z_total_descent += step
        else:
            self.current_z += step
        self.z_wait_counter = wait_cycles
        self._log_seq += 1
        if self._log_seq % 10 == 0:
            label = tag if tag else ('↓' if direction == 'down' else '↑')
            self.get_logger().info(
                f'[Z{label}] {step*1e3:.2f}mm | Z={self.current_z:.6f} | Fz={self.current_fz:.2f}N'
            )

    # ==================== X 轴状态机 ====================
    def _run_x_machine(self):
        if self.x_phase == 'IDLE':
            if self.x_target == 0:
                self.x_phase = 'HOLD'
                return True
            y_ready = (self.y_phase == 'HOLD') or ('y' not in self.active_axes)
            if self.z_phase == 'HOLD' and y_ready and self.z_hold_time > 0:
                if time.time() - self.z_hold_time >= self.x_start_delay:
                    self.get_logger().info(
                        f'[X] Z保压{self.x_start_delay:.0f}s到，启动X轴 '
                        f'(目标Fx={self.x_target:.1f}N sign={self.x_step_sign})'
                    )
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
        error = abs(self.x_target - abs(self.current_fx))
        if error > self.x_drift:
            self.x_prev_fy = abs(self.current_fx)
            self.x_prev_x = self.current_x
            self.x_error_before = error
            direction = 'fwd' if abs(self.current_fx) < self.x_target else 'rev'
            self.get_logger().info(
                f'[X-HOLD] 漂移 |Fx|={abs(self.current_fx):.2f}N err={error:.2f}>{self.x_drift}N → {direction}步进'
            )
            self._x_do_step(self.x_step, self.x_fine_wait, direction=direction)
            self.x_phase = 'RECOVER_WAIT'
            return True
        return False

    def _x_evaluate_approach(self):
        fx_abs = abs(self.current_fx)
        if fx_abs >= self.x_target:
            e_prev = abs(self.x_target - self.x_prev_fy)
            e_curr = abs(self.x_target - fx_abs)
            self.get_logger().info(
                f'[X] 跨越 {self.x_target:.1f}N | '
                f'prev |Fx|={self.x_prev_fy:.2f} err={e_prev:.2f} | curr |Fx|={fx_abs:.2f} err={e_curr:.2f}'
            )
            if e_prev <= e_curr:
                self.current_x = self.x_prev_x
                self.get_logger().info(f'[X] >>> prev更接近 err={e_prev:.2f}≤{e_curr:.2f} → HOLD X={self.current_x:.6f}')
                self.x_phase = 'HOLD'
                self._publish_cartepos()
            else:
                self.x_error_before = e_curr
                self.x_hold_x = self.current_x
                self.get_logger().info(f'[X] curr更接近 err={e_curr:.2f}<{e_prev:.2f} → 精调一步')
                self._x_do_step(self.x_step, self.x_fine_wait)
                self.x_phase = 'FINE_TUNE_WAIT'
            return

        self.x_prev_fy = fx_abs
        self.x_prev_x = self.current_x
        fine_th = self.x_target * self.x_fine_ratio
        if fx_abs < fine_th:
            self._x_do_step(self.x_approach_step, self.x_coarse_wait, tag='X粗')
        else:
            self._x_do_step(self.x_step, self.x_fine_wait, tag='X细')

    def _x_evaluate_fine_tune(self):
        e_after = abs(self.x_target - abs(self.current_fx))
        self.get_logger().info(f'[X] 精调 err {self.x_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.x_error_before:
            self.get_logger().info(f'[X] >>> 精调OK → HOLD X={self.current_x:.6f}')
        else:
            self.current_x = self.x_hold_x
            self.get_logger().info(f'[X] >>> 回退 → HOLD X={self.current_x:.6f}')
            self._publish_cartepos()
        self.x_phase = 'HOLD'

    def _x_evaluate_recover(self):
        e_after = abs(self.x_target - abs(self.current_fx))
        self.get_logger().info(f'[X] 纠偏 err {self.x_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.x_error_before:
            self.get_logger().info(f'[X] >>> 纠偏OK → HOLD')
        else:
            self.current_x = self.x_prev_x
            self.get_logger().info(f'[X] >>> 纠偏无效 回退 → HOLD X={self.current_x:.6f}')
            self._publish_cartepos()
        self.x_phase = 'HOLD'

    def _x_do_step(self, step, wait_cycles, tag='', direction='fwd'):
        if direction == 'fwd':
            self.current_x += self.x_step_sign * step
        else:
            self.current_x -= self.x_step_sign * step
        self.x_wait_counter = wait_cycles
        self._log_seq += 1
        if self._log_seq % 10 == 0:
            label = tag if tag else ('→' if direction == 'fwd' else '←')
            self.get_logger().info(
                f'[X{label}] {step*1e3:.2f}mm | X={self.current_x:.6f} | |Fx|={abs(self.current_fx):.2f}N'
            )

    # ==================== Y 轴状态机 ====================
    def _run_y_machine(self):
        if self.y_phase == 'IDLE':
            if self.y_target == 0:
                self.y_phase = 'HOLD'
                return True
            if self.z_phase == 'HOLD' and self.z_hold_time > 0:
                if time.time() - self.z_hold_time >= self.y_start_delay:
                    self.get_logger().info(
                        f'[Y] Z保压{self.y_start_delay:.0f}s到，启动Y轴 '
                        f'(目标Fy={self.y_target:.1f}N sign={self.y_step_sign})'
                    )
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
        error = abs(self.y_target - abs(self.current_fy))
        if error > self.y_drift:
            self.y_prev_f = abs(self.current_fy)
            self.y_prev_y = self.current_y
            self.y_error_before = error
            direction = 'fwd' if abs(self.current_fy) < self.y_target else 'rev'
            self.get_logger().info(
                f'[Y-HOLD] 漂移 |Fy|={abs(self.current_fy):.2f}N err={error:.2f}>{self.y_drift}N → {direction}步进'
            )
            self._y_do_step(self.y_step, self.y_fine_wait, direction=direction)
            self.y_phase = 'RECOVER_WAIT'
            return True
        return False

    def _y_evaluate_approach(self):
        fy_abs = abs(self.current_fy)
        if fy_abs >= self.y_target:
            e_prev = abs(self.y_target - self.y_prev_f)
            e_curr = abs(self.y_target - fy_abs)
            self.get_logger().info(
                f'[Y] 跨越 {self.y_target:.1f}N | '
                f'prev |Fy|={self.y_prev_f:.2f} err={e_prev:.2f} | curr |Fy|={fy_abs:.2f} err={e_curr:.2f}'
            )
            if e_prev <= e_curr:
                self.current_y = self.y_prev_y
                self.get_logger().info(f'[Y] >>> prev更接近 err={e_prev:.2f}≤{e_curr:.2f} → HOLD Y={self.current_y:.6f}')
                self.y_phase = 'HOLD'
                self._publish_cartepos()
            else:
                self.y_error_before = e_curr
                self.y_hold_y = self.current_y
                self.get_logger().info(f'[Y] curr更接近 err={e_curr:.2f}<{e_prev:.2f} → 精调一步')
                self._y_do_step(self.y_step, self.y_fine_wait)
                self.y_phase = 'FINE_TUNE_WAIT'
            return

        self.y_prev_f = fy_abs
        self.y_prev_y = self.current_y
        fine_th = self.y_target * self.y_fine_ratio
        if fy_abs < fine_th:
            self._y_do_step(self.y_approach_step, self.y_coarse_wait, tag='Y粗')
        else:
            self._y_do_step(self.y_step, self.y_fine_wait, tag='Y细')

    def _y_evaluate_fine_tune(self):
        e_after = abs(self.y_target - abs(self.current_fy))
        self.get_logger().info(f'[Y] 精调 err {self.y_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.y_error_before:
            self.get_logger().info(f'[Y] >>> 精调OK → HOLD Y={self.current_y:.6f}')
        else:
            self.current_y = self.y_hold_y
            self.get_logger().info(f'[Y] >>> 回退 → HOLD Y={self.current_y:.6f}')
            self._publish_cartepos()
        self.y_phase = 'HOLD'

    def _y_evaluate_recover(self):
        e_after = abs(self.y_target - abs(self.current_fy))
        self.get_logger().info(f'[Y] 纠偏 err {self.y_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.y_error_before:
            self.get_logger().info(f'[Y] >>> 纠偏OK → HOLD')
        else:
            self.current_y = self.y_prev_y
            self.get_logger().info(f'[Y] >>> 纠偏无效 回退 → HOLD Y={self.current_y:.6f}')
            self._publish_cartepos()
        self.y_phase = 'HOLD'

    def _y_do_step(self, step, wait_cycles, tag='', direction='fwd'):
        if direction == 'fwd':
            self.current_y += self.y_step_sign * step
        else:
            self.current_y -= self.y_step_sign * step
        self.y_wait_counter = wait_cycles
        self._log_seq += 1
        if self._log_seq % 10 == 0:
            label = tag if tag else ('→' if direction == 'fwd' else '←')
            self.get_logger().info(
                f'[Y{label}] {step*1e3:.2f}mm | Y={self.current_y:.6f} | |Fy|={abs(self.current_fy):.2f}N'
            )

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
            if y_t is not None:
                self.y_target = y_t
                self.y_force_limit = max(y_t * 3.0, 5.0)
            if x_t is not None:
                self.x_target = x_t
                self.x_force_limit = max(x_t * 3.0, 5.0)

            self.get_logger().info(
                f'到达初始位姿 → 网格点 1/{len(self.grid_points)}: '
                f'z={z_t}, y={y_t}, x={x_t}'
            )

            if 'z' in self.active_axes:
                self.z_phase = 'APPROACH'
                self.z_wait_counter = 0
                self.z_total_descent = 0.0
                self.z_prev_fz = self.current_fz
                self.z_prev_z = self.current_z

            if 'y' in self.active_axes and 'z' not in self.active_axes:
                self.y_phase = 'APPROACH'
                self.y_wait_counter = 0
            if 'x' in self.active_axes and 'z' not in self.active_axes:
                self.x_phase = 'APPROACH'
                self.x_wait_counter = 0

            self.state = 'FORCE_CONTROL'
            self.grid_phase = 'FORCE_CONTROL'

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
