#!/usr/bin/env python3
"""
恒力保压控制节点（多轴通用步进试探式）
- 通过 control_axes 参数选择控制轴: "x", "y", "z", "xz", "xy", "xyz"
- 每轴独立参数（步长、目标力、方向、延迟等），完全可自定义
- 时序: MoveJp → Z轴优先下压 → 其余轴按 start_delay 依次激活 → 各轴 HOLD 漂移纠偏
- 订阅 /force_sensor_data，发布 /rm_driver/movej_p_cmd + /rm_driver/movep_canfd_cmd

参数统一管理: config/force_control_params.yaml — 唯一参数来源
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rm_ros_interfaces.msg import Movejp, Cartepos
from ament_index_python.packages import get_package_share_directory
import time


class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control_node')

        # 从 YAML 加载所有参数默认值（唯一参数来源）
        pkg_share = get_package_share_directory('rm_75_calibration')
        yaml_path = os.path.join(pkg_share, 'config', 'force_control_params.yaml')
        with open(yaml_path, 'r') as f:
            yaml_params = yaml.safe_load(f)['/force_control_node']['ros__parameters']
        for key, val in yaml_params.items():
            self.declare_parameter(key, val)

        self._read_params()

        # ========== 姿态 ==========
        self.ori = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]

        # ========== 订阅 ==========
        self.current_fx = 0.0
        self.current_fy = 0.0
        self.current_fz = 0.0
        self.force_sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._force_callback, 10
        )

        # ========== 发布 ==========
        self.movej_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.cartepos_pub = self.create_publisher(Cartepos, '/rm_driver/movep_canfd_cmd', 10)

        # ========== 全局状态 ==========
        self.state = 'MOVE_TO_START'
        self._move_deadline = 0.0
        self._log_seq = 0

        # ========== 每轴状态 ==========
        start_pos = {'x': self.start_x, 'y': self.start_y, 'z': self.start_z}
        self.ax = {}
        for a in self.active_axes:
            self.ax[a] = {
                'phase': 'IDLE',
                'pos': start_pos[a],
                'wait_ctr': 0,
                'prev_force': 0.0,
                'prev_pos': start_pos[a],
                'error_before': 0.0,
                'hold_pos': start_pos[a],
                'hold_time': 0.0,
            }

        # ========== 100Hz 控制定时器 ==========
        time.sleep(3.0)
        self.ctrl_timer = self.create_timer(0.01, self._control_callback)
        self.get_logger().info(
            f'多轴恒力控制节点启动 | 控制轴={self.control_axes} | '
            f'激活轴={self.active_axes}'
        )

    # ==================== 参数读取 ====================
    def _read_params(self):
        self.control_axes = self.get_parameter('control_axes').value
        self.active_axes = list(self.control_axes)
        self.movej_speed = self.get_parameter('movej_speed').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.ori_x = self.get_parameter('orientation_x').value
        self.ori_y = self.get_parameter('orientation_y').value
        self.ori_z = self.get_parameter('orientation_z').value
        self.ori_w = self.get_parameter('orientation_w').value
        self.trajectory_duration = self.get_parameter('trajectory_duration').value
        self.hold_duration = self.get_parameter('hold_duration').value

        self.cfg = {}
        for a in ('x', 'y', 'z'):
            self.cfg[a] = {
                'target': self.get_parameter(f'{a}_target_force').value,
                'step': self.get_parameter(f'{a}_step_size').value,
                'approach': self.get_parameter(f'{a}_approach_step_size').value,
                'fine_ratio': self.get_parameter(f'{a}_fine_threshold_ratio').value,
                'coarse_wait': self.get_parameter(f'{a}_coarse_stabilize_cycles').value,
                'fine_wait': self.get_parameter(f'{a}_fine_stabilize_cycles').value,
                'drift': self.get_parameter(f'{a}_drift_threshold').value,
                'step_sign': self.get_parameter(f'{a}_step_sign').value,
                'start_delay': self.get_parameter(f'{a}_start_delay').value,
                'force_field': self.get_parameter(f'{a}_force_field').value,
                'force_sign': self.get_parameter(f'{a}_force_sign').value,
            }
            self.cfg[a]['force_limit'] = (
                self.cfg[a]['target'] * 3.0 if a in ('x', 'y')
                else self.cfg[a]['target'] * 1.5
            )

    # ==================== 力传感器回调 ====================
    def _force_callback(self, msg):
        self.current_fx = msg.wrench.force.x
        self.current_fy = msg.wrench.force.y
        self.current_fz = msg.wrench.force.z

    def _get_force(self, axis):
        """返回带符号的当前力值"""
        raw = {'x': self.current_fx, 'y': self.current_fy, 'z': self.current_fz}[axis]
        return raw * self.cfg[axis]['force_sign']

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
        for a in self.active_axes:
            f = abs(self._get_force(a))
            if f > self.cfg[a]['force_limit']:
                self.get_logger().error(
                    f'[{a.upper()}] 力超限 force={f:.2f}N > {self.cfg[a]["force_limit"]}N，紧急停止！'
                )
                self.state = 'ESTOP'
                return

        # ===== 运行每轴状态机（优先级: z → x → y） =====
        activation_order = sorted(self.active_axes, key=lambda a: {'z': 0, 'x': 1, 'y': 2}[a])
        any_publish = False
        for a in activation_order:
            if self._run_axis_machine(a):
                any_publish = True

        if any_publish:
            self._publish_cartepos()

        # ===== 检查全部轴 HOLD → 可选的持续保压时间 → DONE =====
        if self.hold_duration > 0:
            all_hold = all(self.ax[a]['phase'] == 'HOLD' for a in self.active_axes)
            if all_hold:
                earliest_hold = min(self.ax[a]['hold_time'] for a in self.active_axes)
                if time.time() - earliest_hold >= self.hold_duration:
                    self.get_logger().info(
                        f'保压时长 {self.hold_duration:.1f}s 已达到，停止控制'
                    )
                    self.state = 'DONE'

    # ==================== 单轴状态机 ====================
    def _run_axis_machine(self, axis):
        """返回是否需要发布 Cartepos"""
        st = self.ax[axis]

        if st['phase'] == 'IDLE':
            return self._axis_check_activation(axis)

        if st['phase'] == 'HOLD':
            return self._axis_hold_monitor(axis)

        # 活跃阶段：步间等待
        if st['wait_ctr'] > 0:
            st['wait_ctr'] -= 1
            return True

        if st['phase'] == 'APPROACH':
            self._axis_evaluate_approach(axis)
        elif st['phase'] == 'FINE_TUNE_WAIT':
            self._axis_evaluate_fine_tune(axis)
        elif st['phase'] == 'RECOVER_WAIT':
            self._axis_evaluate_recover(axis)
        return True

    def _axis_check_activation(self, axis):
        """检查是否满足激活条件（前置轴已 HOLD 且延迟已过）"""
        cfg = self.cfg[axis]

        # 确定前置轴：z 无前置，x/y 的前置是 z
        prereq = {'z': None, 'x': 'z', 'y': 'z'}[axis]

        if prereq and prereq in self.ax:
            if self.ax[prereq]['phase'] != 'HOLD':
                return False

        # 如果有延迟，检查时间
        if cfg['start_delay'] > 0 and prereq and prereq in self.ax:
            if time.time() - self.ax[prereq]['hold_time'] < cfg['start_delay']:
                return False

        self.get_logger().info(
            f'[{axis.upper()}] 启动 | 目标={cfg["target"]:.1f}N '
            f'步长={cfg["step"]*1e3:.2f}mm sign={cfg["step_sign"]}'
        )
        st = self.ax[axis]
        st['phase'] = 'APPROACH'
        st['wait_ctr'] = 0
        st['prev_force'] = abs(self._get_force(axis))
        st['prev_pos'] = st['pos']
        return True

    # ---------- 逼近评估 ----------
    def _axis_evaluate_approach(self, axis):
        cfg = self.cfg[axis]
        st = self.ax[axis]
        f = self._get_force(axis)
        f_abs = abs(f)

        if f_abs >= cfg['target']:
            e_prev = abs(cfg['target'] - st['prev_force'])
            e_curr = abs(cfg['target'] - f_abs)
            self.get_logger().info(
                f'[{axis.upper()}] 跨越 {cfg["target"]:.1f}N | '
                f'prev={st["prev_force"]:.2f} err={e_prev:.2f} | curr={f_abs:.2f} err={e_curr:.2f}'
            )
            if e_prev <= e_curr:
                st['pos'] = st['prev_pos']
                self.get_logger().info(
                    f'[{axis.upper()}] >>> prev更接近 → HOLD pos={st["pos"]:.6f}'
                )
                st['phase'] = 'HOLD'
                st['hold_time'] = time.time()
                self._publish_cartepos()
            else:
                st['error_before'] = e_curr
                st['hold_pos'] = st['pos']
                self.get_logger().info(f'[{axis.upper()}] curr更接近 → 精调一步')
                self._axis_do_step(axis, cfg['step'], cfg['fine_wait'])
                st['phase'] = 'FINE_TUNE_WAIT'
            return

        st['prev_force'] = f_abs
        st['prev_pos'] = st['pos']
        fine_th = cfg['target'] * cfg['fine_ratio']
        if f_abs < fine_th:
            self._axis_do_step(axis, cfg['approach'], cfg['coarse_wait'], tag='粗')
        else:
            self._axis_do_step(axis, cfg['step'], cfg['fine_wait'], tag='细')

    # ---------- 精调评估 ----------
    def _axis_evaluate_fine_tune(self, axis):
        st = self.ax[axis]
        cfg = self.cfg[axis]
        e_after = abs(cfg['target'] - abs(self._get_force(axis)))
        self.get_logger().info(
            f'[{axis.upper()}] 精调 err {st["error_before"]:.2f}→{e_after:.2f}N'
        )
        if e_after <= st['error_before']:
            self.get_logger().info(f'[{axis.upper()}] >>> 精调OK → HOLD')
        else:
            st['pos'] = st['hold_pos']
            self.get_logger().info(f'[{axis.upper()}] >>> 回退 → HOLD pos={st["pos"]:.6f}')
            self._publish_cartepos()
        st['phase'] = 'HOLD'
        st['hold_time'] = time.time()

    # ---------- 纠偏评估 ----------
    def _axis_evaluate_recover(self, axis):
        st = self.ax[axis]
        cfg = self.cfg[axis]
        e_after = abs(cfg['target'] - abs(self._get_force(axis)))
        self.get_logger().info(
            f'[{axis.upper()}] 纠偏 err {st["error_before"]:.2f}→{e_after:.2f}N'
        )
        if e_after <= st['error_before']:
            self.get_logger().info(f'[{axis.upper()}] >>> 纠偏OK → HOLD')
        else:
            st['pos'] = st['prev_pos']
            self.get_logger().info(f'[{axis.upper()}] >>> 无效回退 → HOLD pos={st["pos"]:.6f}')
            self._publish_cartepos()
        st['phase'] = 'HOLD'

    # ---------- HOLD 漂移监控 ----------
    def _axis_hold_monitor(self, axis):
        cfg = self.cfg[axis]
        st = self.ax[axis]
        f_abs = abs(self._get_force(axis))
        error = abs(cfg['target'] - f_abs)

        if error > cfg['drift']:
            st['prev_force'] = f_abs
            st['prev_pos'] = st['pos']
            st['error_before'] = error
            direction = 'fwd' if f_abs < cfg['target'] else 'rev'
            self.get_logger().info(
                f'[{axis.upper()}-HOLD] 漂移 force={f_abs:.2f}N '
                f'err={error:.2f}>{cfg["drift"]}N → {direction}步进'
            )
            self._axis_do_step(axis, cfg['step'], cfg['fine_wait'], direction=direction)
            st['phase'] = 'RECOVER_WAIT'
            return True

        return False

    # ---------- 单步执行 ----------
    def _axis_do_step(self, axis, step, wait_cycles, direction='fwd', tag=''):
        cfg = self.cfg[axis]
        st = self.ax[axis]

        if direction == 'fwd':
            st['pos'] += cfg['step_sign'] * step
        else:
            st['pos'] -= cfg['step_sign'] * step

        st['wait_ctr'] = wait_cycles
        self._log_seq += 1
        if self._log_seq % 10 == 0:
            label = tag if tag else ('→' if direction == 'fwd' else '←')
            self.get_logger().info(
                f'[{axis.upper()}{label}] {step*1e3:.2f}mm | '
                f'pos={st["pos"]:.6f} | force={abs(self._get_force(axis)):.2f}N'
            )

    # ==================== 初始定位 ====================
    def _enter_move_to_start(self):
        self.get_logger().info(
            f'MoveJp → ({self.start_x:.4f}, {self.start_y:.4f}, {self.start_z:.4f})'
        )
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
        for a in self.active_axes:
            self.ax[a]['pos'] = {'x': self.start_x, 'y': self.start_y, 'z': self.start_z}[a]
        self._move_deadline = time.time() + 5.0
        self.state = 'WAIT_MOVEJ'

    def _check_movej_done(self):
        if time.time() >= self._move_deadline:
            # 按优先级激活第一个轴（z > x > y）
            first = sorted(self.active_axes, key=lambda a: {'z': 0, 'x': 1, 'y': 2}[a])[0]
            cfg = self.cfg[first]
            self.get_logger().info(
                f'到达初始位姿 → 启动 [{first.upper()}] 轴 '
                f'(目标={cfg["target"]:.1f}N)'
            )
            st = self.ax[first]
            st['phase'] = 'APPROACH'
            st['wait_ctr'] = 0
            st['prev_force'] = abs(self._get_force(first))
            st['prev_pos'] = st['pos']
            self.state = 'FORCE_CONTROL'

    # ==================== 发布 Cartepos ====================
    def _publish_cartepos(self):
        msg = Cartepos()
        msg.pose.position.x = float(self.ax['x']['pos'] if 'x' in self.ax else self.start_x)
        msg.pose.position.y = float(self.ax['y']['pos'] if 'y' in self.ax else self.start_y)
        msg.pose.position.z = float(self.ax['z']['pos'] if 'z' in self.ax else self.start_z)
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
