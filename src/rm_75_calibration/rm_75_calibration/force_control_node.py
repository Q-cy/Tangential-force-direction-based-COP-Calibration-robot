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
from std_msgs.msg import String
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
        self.state_pub = self.create_publisher(String, '/force_control_state', 10)

        # ========== 订阅返回初始点命令 ==========
        self.return_home_sub = self.create_subscription(
            String, '/force_control_return_home', self._return_home_callback, 10)

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
                'next_checkpoint': 0.0,
                'drift_boosted': False,
                'backoff_attempts': 0,
                'best_err': 1e9,
                'best_pos': start_pos[a],
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
                'force_step': self.get_parameter(f'{a}_force_step').value,
                'step_dwell': self.get_parameter(f'{a}_step_dwell').value,
                'tolerance': self.get_parameter(f'{a}_force_tolerance').value,
                'cross_drift_mul': self.get_parameter(f'{a}_cross_axis_drift_multiplier').value,
                'control_mode': str(self.get_parameter(f'{a}_control_mode').value),
                'unload_dwell': self.get_parameter(f'{a}_unload_dwell').value,
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
            self._enter_return_home()
            return
        if self.state == 'DONE':
            self._enter_return_home()
            return
        if self.state == 'RETURN_HOME':
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
            prev_phase = self.ax[a]['phase']
            if self._run_axis_machine(a):
                any_publish = True
            # 本轴刚进入 HOLD → 清除其他轴的 drift_boosted
            if prev_phase != 'HOLD' and self.ax[a]['phase'] == 'HOLD':
                for other in activation_order:
                    if other != a:
                        self.ax[other]['drift_boosted'] = False

        if any_publish:
            self._publish_cartepos()

        # 发布当前各轴 phase（供 realtime 进程判断有效数据）
        phases = ','.join(f'{a}:{self.ax[a]["phase"]}' for a in activation_order)
        self.state_pub.publish(String(data=phases))

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
        elif st['phase'] == 'CHECKPOINT_BACKOFF':
            self._axis_checkpoint_backoff(axis)
        elif st['phase'] == 'STEP_DWELL':
            self._axis_step_dwell(axis)
        elif st['phase'] == 'LOAD_HOLD':
            self._axis_load_hold(axis)
        elif st['phase'] == 'UNLOAD':
            self._axis_do_unload(axis)
        elif st['phase'] == 'UNLOAD_DWELL':
            self._axis_unload_dwell(axis)
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
        # 放大其他 HOLD 轴的漂移阈值，避免交叉耦合导致误纠偏
        for a in self.active_axes:
            if a != axis and self.ax[a]['phase'] == 'HOLD':
                self.ax[a]['drift_boosted'] = True
        st = self.ax[axis]
        st['phase'] = 'APPROACH'
        st['wait_ctr'] = 0
        st['prev_force'] = abs(self._get_force(axis))
        st['prev_pos'] = st['pos']
        st['next_checkpoint'] = cfg.get('force_step', 0)
        return True

    # ---------- 逼近评估 ----------
    def _axis_evaluate_approach(self, axis):
        cfg = self.cfg[axis]
        st = self.ax[axis]
        f = self._get_force(axis)
        f_abs = abs(f)
        tol = cfg.get('tolerance', 0.1)

        # 力值阶梯模式：容差到达 + crossing 逻辑（与最终目标一致）
        force_step = cfg.get('force_step', 0)
        if force_step > 0 and abs(f_abs - st['next_checkpoint']) <= tol:
            e_prev = abs(st['next_checkpoint'] - st['prev_force'])
            e_curr = abs(st['next_checkpoint'] - f_abs)
            self.get_logger().info(
                f'[{axis.upper()}] 阶梯 {st["next_checkpoint"]:.1f}N 到达 | '
                f'prev={st["prev_force"]:.2f} err={e_prev:.2f} | curr={f_abs:.2f} err={e_curr:.2f}'
            )
            if e_prev <= e_curr:
                st['pos'] = st['prev_pos']
                self.get_logger().info(f'[{axis.upper()}] >>> prev更接近 → 回退 + 静置')
                self._publish_cartepos()
            else:
                self.get_logger().info(f'[{axis.upper()}] >>> curr更接近 → 静置')
            dwell = cfg.get('step_dwell', 50)
            if cfg.get('control_mode', 'staircase') == 'load_unload':
                st['phase'] = 'LOAD_HOLD'
            else:
                st['phase'] = 'STEP_DWELL'
            st['wait_ctr'] = dwell
            self._publish_cartepos()
            return

        # 最终目标检测（容差 + crossing 逻辑）
        if abs(f_abs - cfg['target']) <= tol:
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

        # 过冲检测：力值已远超下一个台阶 → 退回调优
        if force_step > 0 and f_abs > st['next_checkpoint'] + tol:
            st['backoff_attempts'] = 0
            st['best_err'] = 1e9
            st['best_pos'] = st['pos']
            self.get_logger().info(
                f'[{axis.upper()}] 过冲检测：force={f_abs:.2f} > '
                f'checkpoint={st["next_checkpoint"]:.1f}+tol={tol:.1f} → 退回调优'
            )
            st['phase'] = 'CHECKPOINT_BACKOFF'
            self._axis_checkpoint_backoff(axis)
            return

        st['prev_force'] = f_abs
        st['prev_pos'] = st['pos']
        fine_th = cfg['target'] * cfg['fine_ratio']
        if f_abs < fine_th:
            self._axis_do_step(axis, cfg['approach'], cfg['coarse_wait'], tag='粗')
        else:
            self._axis_do_step(axis, cfg['step'], cfg['fine_wait'], tag='细')

    # ---------- 过冲退回调优 ----------
    def _axis_checkpoint_backoff(self, axis):
        st = self.ax[axis]
        cfg = self.cfg[axis]
        f_abs = abs(self._get_force(axis))
        tol = cfg.get('tolerance', 0.1)

        err = abs(f_abs - st['next_checkpoint'])
        if err < st['best_err']:
            st['best_err'] = err
            st['best_pos'] = st['pos']
        st['backoff_attempts'] += 1

        # 进入容差 → 成功
        if err <= tol:
            self.get_logger().info(
                f'[{axis.upper()}] 回退成功：force={f_abs:.2f}N → 台阶 {st["next_checkpoint"]:.1f}N'
            )
            st['phase'] = 'STEP_DWELL'
            st['wait_ctr'] = cfg.get('step_dwell', 50)
            self._publish_cartepos()
            return

        # 超过最大尝试次数 → 取最接近位置
        if st['backoff_attempts'] >= 10:
            st['pos'] = st['best_pos']
            self.get_logger().info(
                f'[{axis.upper()}] 回退超限 → 取最佳位置 pos={st["pos"]:.6f} '
                f'(best_err={st["best_err"]:.2f}N)'
            )
            st['phase'] = 'STEP_DWELL'
            st['wait_ctr'] = cfg.get('step_dwell', 50)
            self._publish_cartepos()
            return

        # 反向精细步进
        self._axis_do_step(axis, cfg['step'], cfg['fine_wait'], direction='rev', tag='回退')

    # ---------- 台阶静置 ----------
    def _axis_step_dwell(self, axis):
        st = self.ax[axis]
        cfg = self.cfg[axis]
        force_step = cfg.get('force_step', 0)
        st['next_checkpoint'] += force_step
        if st['next_checkpoint'] >= cfg['target']:
            self.get_logger().info(
                f'[{axis.upper()}] 台阶静置完成 → 最终目标 {cfg["target"]:.1f}N 已到达，进入 HOLD'
            )
            st['phase'] = 'HOLD'
            st['hold_time'] = time.time()
            self._publish_cartepos()
        else:
            self.get_logger().info(
                f'[{axis.upper()}] 台阶静置完成 → 下一台阶 {st["next_checkpoint"]:.1f}N'
            )
            st['phase'] = 'APPROACH'
            st['wait_ctr'] = 0

    # ---------- 加载-卸载模式：保压 ----------
    def _axis_load_hold(self, axis):
        st = self.ax[axis]
        st['phase'] = 'UNLOAD'
        st['wait_ctr'] = 0
        st['unload_start_pos'] = st['pos']
        self.get_logger().info(f'[{axis.upper()}] 保压完成 → 开始卸载')

    # ---------- 加载-卸载模式：抬起卸载 ----------
    def _axis_do_unload(self, axis):
        cfg = self.cfg[axis]
        st = self.ax[axis]
        f_abs = abs(self._get_force(axis))

        if f_abs <= cfg.get('tolerance', 0.1):
            dwell = cfg.get('unload_dwell', 500)
            self.get_logger().info(
                f'[{axis.upper()}] 卸载完成 force={f_abs:.2f}N → 静置 {dwell} 周期'
            )
            st['phase'] = 'UNLOAD_DWELL'
            st['wait_ctr'] = dwell
            return

        self._axis_do_step(axis, cfg['approach'], cfg['coarse_wait'], direction='rev', tag='卸载')

    # ---------- 加载-卸载模式：卸载后静置 ----------
    def _axis_unload_dwell(self, axis):
        st = self.ax[axis]
        st['next_checkpoint'] += self.cfg[axis].get('force_step', 0)
        if st['next_checkpoint'] >= self.cfg[axis]['target']:
            self.get_logger().info(
                f'[{axis.upper()}] 全部台阶完成 → 最终目标 {self.cfg[axis]["target"]:.1f}N'
            )
            st['phase'] = 'HOLD'
            st['hold_time'] = time.time()
            self._publish_cartepos()
        else:
            self.get_logger().info(
                f'[{axis.upper()}] 卸载静置完成 → 下一台阶 {st["next_checkpoint"]:.1f}N'
            )
            st['phase'] = 'APPROACH'
            st['wait_ctr'] = 0

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
        effective_drift = cfg['drift'] * (cfg.get('cross_drift_mul', 3.0) if st.get('drift_boosted') else 1.0)

        if error > effective_drift:
            st['prev_force'] = f_abs
            st['prev_pos'] = st['pos']
            st['error_before'] = error
            direction = 'fwd' if f_abs < cfg['target'] else 'rev'
            self.get_logger().info(
                f'[{axis.upper()}-HOLD] 漂移 force={f_abs:.2f}N '
                f'err={error:.2f}>{effective_drift:.2f}N → {direction}步进'
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

    # ==================== 返回初始点 ====================
    def _return_home_callback(self, msg):
        self._enter_return_home()

    def _enter_return_home(self):
        if self.state == 'RETURN_HOME':
            return
        self.get_logger().info(
            f'返回初始点 → ({self.start_x:.4f}, {self.start_y:.4f}, {self.start_z:.4f})'
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
        self.state = 'RETURN_HOME'

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
        node.get_logger().info('用户中断，返回初始点...')
        node._enter_return_home()
        time.sleep(6)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
