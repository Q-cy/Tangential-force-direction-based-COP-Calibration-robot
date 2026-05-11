#!/usr/bin/env python3
"""
恒力保压控制节点（双轴步进试探式）
- Z 轴：法向力 Fz 保压控制（步进试探），默认 10N
- X 轴：切向力 Fy 保压控制（步进试探），默认 1N
- 时序：MoveJp 到位 → Z 轴下压至目标 → Z 保压 10s → X 轴横移保压
- X 轴运动期间 Z 轴持续监控漂移纠偏
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rm_ros_interfaces.msg import Movejp, Cartepos
import time


class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control_xz_node')

        # ==================== Z 轴参数（法向力 Fz） ====================
        self.declare_parameter('z_target_force', 20.0)
        self.declare_parameter('z_step_size', 0.00005) #0.05mm
        self.declare_parameter('z_approach_step_size', 0.0001) #0.1mm
        self.declare_parameter('z_fine_threshold_ratio', 0.8)
        self.declare_parameter('z_coarse_stabilize_cycles', 10)
        self.declare_parameter('z_fine_stabilize_cycles', 50)
        self.declare_parameter('z_drift_threshold', 0.2)

        # ==================== X 轴参数（切向力 Fy） ====================
        self.declare_parameter('x_target_force', 5.0)
        self.declare_parameter('x_step_size', 0.00005) #0.05mm
        self.declare_parameter('x_approach_step_size', 0.0001) #0.1mm
        self.declare_parameter('x_fine_threshold_ratio', 0.8)
        self.declare_parameter('x_coarse_stabilize_cycles', 10)
        self.declare_parameter('x_fine_stabilize_cycles', 50)
        self.declare_parameter('x_drift_threshold', 0.2)
        self.declare_parameter('x_step_sign', 1)   # -1: X- 增加 |Fy|, +1: X+ 增加 |Fy|
        self.declare_parameter('x_start_delay', 10.0)  # Z 保压后等待多久启动 X

        # ==================== 通用参数 ====================
        self.declare_parameter('movej_speed', 15)
        self.declare_parameter('start_x', 0.432)
        self.declare_parameter('start_y', -0.0065)
        self.declare_parameter('start_z', 0.18)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 1.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 0.0)

        self._read_params()

        # ========== 姿态 ==========
        self.ori = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]

        # ========== 订阅 ==========
        self.current_fz = 0.0
        self.current_fy = 0.0
        self.force_sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._force_callback, 10
        )

        # ========== 发布 ==========
        self.movej_pub = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.cartepos_pub = self.create_publisher(Cartepos, '/rm_driver/movep_canfd_cmd', 10)

        # ==================== Z 轴状态 ====================
        self.state = 'MOVE_TO_START'
        self.current_z = self.start_z
        self.z_phase = 'APPROACH'
        self.z_wait_counter = 0
        self.z_prev_fz = 0.0
        self.z_prev_z = self.start_z
        self.z_error_before = 0.0
        self.z_hold_z = 0.0
        self.z_total_descent = 0.0
        self.z_hold_time = 0.0

        # ==================== X 轴状态 ====================
        self.current_x = self.start_x
        self.x_phase = 'IDLE'        # IDLE → APPROACH → FINE_TUNE_WAIT → RECOVER_WAIT → HOLD
        self.x_wait_counter = 0
        self.x_prev_fy = 0.0
        self.x_prev_x = self.start_x
        self.x_error_before = 0.0
        self.x_hold_x = 0.0

        # ========== 日志 ==========
        self._log_seq = 0

        # ========== 100Hz 控制定时器 ==========
        time.sleep(3.0)
        self.ctrl_timer = self.create_timer(0.01, self._control_callback)
        self.get_logger().info('恒力保压控制节点启动（双轴步进试探模式）')

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
        self.x_force_limit = self.x_target * 3.0
        # common
        self.movej_speed = self.get_parameter('movej_speed').value
        self.start_x = self.get_parameter('start_x').value
        self.start_y = self.get_parameter('start_y').value
        self.start_z = self.get_parameter('start_z').value
        self.ori_x = self.get_parameter('orientation_x').value
        self.ori_y = self.get_parameter('orientation_y').value
        self.ori_z = self.get_parameter('orientation_z').value
        self.ori_w = self.get_parameter('orientation_w').value

    # ==================== 力传感器回调 ====================
    def _force_callback(self, msg):
        self.current_fz = -msg.wrench.force.z
        self.current_fy = msg.wrench.force.y

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

        # ===== 安全检查 =====
        if abs(self.current_fz) > self.z_force_limit:
            self.get_logger().error(f'Z力超限 Fz={self.current_fz:.2f}>{self.z_force_limit}N')
            self.state = 'ESTOP'; return
        if abs(self.current_fy) > self.x_force_limit:
            self.get_logger().error(f'X力超限 Fy={self.current_fy:.2f}>{self.x_force_limit}N')
            self.state = 'ESTOP'; return

        # ===== 先跑 Z 轴状态机 =====
        z_publish = self._run_z_machine()

        # ===== 再跑 X 轴状态机（Z HOLD 之后才激活） =====
        x_publish = self._run_x_machine()

        # ===== 任一轴活跃则发布 Cartepos =====
        if z_publish or x_publish:
            self._publish_cartepos()

    # ==================== Z 轴状态机 ====================
    def _run_z_machine(self):
        """返回是否需要发布 Cartepos"""
        if self.z_phase == 'HOLD':
            return self._z_hold_monitor()

        # 活跃阶段
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
        """返回是否需要发布 Cartepos。Z HOLD 前 X 保持 IDLE。"""
        if self.x_phase == 'IDLE':
            if self.z_phase == 'HOLD' and self.z_hold_time > 0:
                if time.time() - self.z_hold_time >= self.x_start_delay:
                    self.get_logger().info(
                        f'[X] Z保压{self.x_start_delay:.0f}s到，启动X轴 '
                        f'(目标Fy={self.x_target:.1f}N sign={self.x_step_sign})'
                    )
                    self.x_phase = 'APPROACH'
                    self.x_wait_counter = 0
                    self.x_prev_fy = abs(self.current_fy)
                    self.x_prev_x = self.current_x
                    # fall through to active publish
                else:
                    return False
            else:
                return False

        if self.x_phase == 'HOLD':
            return self._x_hold_monitor()

        # 活跃阶段
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
        error = abs(self.x_target - abs(self.current_fy))
        if error > self.x_drift:
            self.x_prev_fy = abs(self.current_fy)
            self.x_prev_x = self.current_x
            self.x_error_before = error
            direction = 'fwd' if abs(self.current_fy) < self.x_target else 'rev'
            self.get_logger().info(
                f'[X-HOLD] 漂移 |Fy|={abs(self.current_fy):.2f}N err={error:.2f}>{self.x_drift}N → {direction}步进'
            )
            self._x_do_step(self.x_step, self.x_fine_wait, direction=direction)
            self.x_phase = 'RECOVER_WAIT'
            return True
        return False

    def _x_evaluate_approach(self):
        fy_abs = abs(self.current_fy)
        if fy_abs >= self.x_target:
            e_prev = abs(self.x_target - self.x_prev_fy)
            e_curr = abs(self.x_target - fy_abs)
            self.get_logger().info(
                f'[X] 跨越 {self.x_target:.1f}N | '
                f'prev |Fy|={self.x_prev_fy:.2f} err={e_prev:.2f} | curr |Fy|={fy_abs:.2f} err={e_curr:.2f}'
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

        self.x_prev_fy = fy_abs
        self.x_prev_x = self.current_x
        fine_th = self.x_target * self.x_fine_ratio
        if fy_abs < fine_th:
            self._x_do_step(self.x_approach_step, self.x_coarse_wait, tag='X粗')
        else:
            self._x_do_step(self.x_step, self.x_fine_wait, tag='X细')

    def _x_evaluate_fine_tune(self):
        e_after = abs(self.x_target - abs(self.current_fy))
        self.get_logger().info(f'[X] 精调 err {self.x_error_before:.2f}→{e_after:.2f}N')
        if e_after <= self.x_error_before:
            self.get_logger().info(f'[X] >>> 精调OK → HOLD X={self.current_x:.6f}')
        else:
            self.current_x = self.x_hold_x
            self.get_logger().info(f'[X] >>> 回退 → HOLD X={self.current_x:.6f}')
            self._publish_cartepos()
        self.x_phase = 'HOLD'

    def _x_evaluate_recover(self):
        e_after = abs(self.x_target - abs(self.current_fy))
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
                f'[X{label}] {step*1e3:.2f}mm | X={self.current_x:.6f} | |Fy|={abs(self.current_fy):.2f}N'
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
        self._move_deadline = time.time() + 5.0
        self.state = 'WAIT_MOVEJ'

    def _check_movej_done(self):
        if time.time() >= self._move_deadline:
            self.get_logger().info(
                f'到达初始位姿 → Z轴步进下压 (目标Fz={self.z_target:.1f}N)'
            )
            self.z_phase = 'APPROACH'
            self.z_wait_counter = 0
            self.z_total_descent = 0.0
            self.z_prev_fz = self.current_fz
            self.z_prev_z = self.current_z
            self.state = 'FORCE_CONTROL'

    # ==================== 发布 Cartepos ====================
    def _publish_cartepos(self):
        msg = Cartepos()
        msg.pose.position.x = float(self.current_x)
        msg.pose.position.y = float(self.start_y)
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
