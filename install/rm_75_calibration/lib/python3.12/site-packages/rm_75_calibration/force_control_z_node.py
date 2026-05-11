#!/usr/bin/env python3
"""
恒力保压控制节点（步进试探式）
- 订阅 /force_sensor_data 获取六维力反馈
- MoveJp 关节空间规划到达初始位姿
- Cartepos 100Hz 持续透传，逐步下压（0.05mm/步），等待稳定后再决策
- 到达目标力值后，再下压一步比对误差：
  误差更小 → 结束，保持原位
  误差增大 → 回退一步，保持原位
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from rm_ros_interfaces.msg import Movejp, Cartepos
import time


class ForceControlNode(Node):
    def __init__(self):
        super().__init__('force_control_z_node')

        # ========== 参数 ==========
        self.declare_parameter('target_force', 10.0)            # 目标力值 (N)
        self.declare_parameter('step_size', 0.00005)           # 精细步长 0.05mm
        self.declare_parameter('approach_step_size', 0.0001)   # 粗调大步长 0.1mm
        self.declare_parameter('fine_threshold_ratio', 0.8)    # 精细阶段切换比例（80%目标力值）
        self.declare_parameter('coarse_stabilize_cycles', 10)  # 粗调稳定等待（100Hz下10=0.1s）
        self.declare_parameter('fine_stabilize_cycles', 50)    # 精细稳定等待（100Hz下50=0.5s）
        self.declare_parameter('drift_threshold', 0.2)         # HOLD 漂移纠偏阈值 (N)
        self.declare_parameter('movej_speed', 20)

        self.declare_parameter('start_x', 0.432)
        self.declare_parameter('start_y', -0.0065)
        self.declare_parameter('start_z', 0.19)
        self.declare_parameter('orientation_x', 0.0)
        self.declare_parameter('orientation_y', 1.0)
        self.declare_parameter('orientation_z', 0.0)
        self.declare_parameter('orientation_w', 0.0)

        self._read_params()

        # ========== 姿态 ==========
        self.ori = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]

        # ========== 订阅 ==========
        self.current_fz = 0.0
        self.force_sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._force_callback, 10
        )

        # ========== 发布 ==========
        self.movej_pub = self.create_publisher(
            Movejp, '/rm_driver/movej_p_cmd', 10
        )
        self.cartepos_pub = self.create_publisher(
            Cartepos, '/rm_driver/movep_canfd_cmd', 10
        )

        # ========== 步进状态 ==========
        self.state = 'MOVE_TO_START'
        self.current_z = self.start_z
        self.phase = 'APPROACH'          # APPROACH | FINE_TUNE_WAIT | RECOVER_WAIT | HOLD
        self.step_wait_counter = 0       # 步间稳定等待剩余周期
        self.prev_fz = 0.0               # 上一步稳定后的 Fz
        self.prev_z = self.start_z       # 上一步的 Z 位置
        self.error_before = 0.0          # 精调前误差
        self.hold_z = 0.0                # 精调前 Z 位置（用于回退）
        self.total_descent = 0.0         # 累计下压距离
        self._log_seq = 0

        # ========== 100Hz 控制定时器 ==========
        self.ctrl_timer = self.create_timer(0.01, self._control_callback)

        self.get_logger().info('恒力保压控制节点启动（步进试探模式）')

    def _read_params(self):
        self.target_force = self.get_parameter('target_force').value
        self.step_size = self.get_parameter('step_size').value
        self.approach_step_size = self.get_parameter('approach_step_size').value
        self.fine_threshold_ratio = self.get_parameter('fine_threshold_ratio').value
        self.coarse_stabilize_cycles = self.get_parameter('coarse_stabilize_cycles').value
        self.fine_stabilize_cycles = self.get_parameter('fine_stabilize_cycles').value
        self.drift_threshold = self.get_parameter('drift_threshold').value
        self.force_limit = self.target_force * 1.5
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
        if abs(self.current_fz) > self.force_limit:
            self.get_logger().error(
                f'力值超限 Fz={self.current_fz:.2f}N > {self.force_limit}N，紧急停止！'
            )
            self.state = 'ESTOP'
            return

        # ===== HOLD：停止发送指令，监控漂移 =====
        if self.phase == 'HOLD':
            error = abs(self.target_force - self.current_fz)
            if error > self.drift_threshold:
                # 漂移超限 → 记录当前位置，步进纠偏
                self.prev_fz = self.current_fz
                self.prev_z = self.current_z
                self.error_before = error
                if self.current_fz < self.target_force:
                    direction = 'down'
                else:
                    direction = 'up'
                self.get_logger().info(
                    f'[HOLD] 漂移超限 Fz={self.current_fz:.2f}N err={error:.2f}N > {self.drift_threshold}N，'
                    f'向{direction}步进纠偏...'
                )
                self._do_step(self.step_size, self.fine_stabilize_cycles, direction=direction)
                self.phase = 'RECOVER_WAIT'
                # fall through to publish + wait below
            else:
                self._log_seq += 1
                if self._log_seq % 200 == 0:
                    self.get_logger().info(
                        f'[HOLD] Fz={self.current_fz:.2f}N err={error:.2f}N | '
                        f'Z={self.current_z:.6f} | 保压中'
                    )
                return

        # ===== 活跃阶段：100Hz 发布 Cartepos 维持当前位置 =====
        self._publish_cartepos()

        # ===== 步间等待：倒计时，归零后评估并决策下一步 =====
        if self.step_wait_counter > 0:
            self.step_wait_counter -= 1
            return

        # 稳定时间到，根据阶段决策
        if self.phase == 'APPROACH':
            self._evaluate_approach()
        elif self.phase == 'FINE_TUNE_WAIT':
            self._evaluate_fine_tune()
        elif self.phase == 'RECOVER_WAIT':
            self._evaluate_recover()

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
        msg.block = True
        self.movej_pub.publish(msg)
        self.current_z = self.start_z
        self._move_deadline = time.time() + 3.0
        self.state = 'WAIT_MOVEJ'

    def _check_movej_done(self):
        if time.time() >= self._move_deadline:
            self.get_logger().info(
                f'到达初始位姿，开始步进下压 '
                f'(目标={self.target_force:.1f}N, 步长={self.step_size*1e3:.2f}mm)'
            )
            self.phase = 'APPROACH'
            self.step_wait_counter = 0
            self.total_descent = 0.0
            self.prev_fz = self.current_fz
            self.prev_z = self.current_z
            self.state = 'FORCE_CONTROL'

    # ==================== 步进下压（APPROACH） ====================
    def _evaluate_approach(self):
        if self.current_fz >= self.target_force:
            # 跨越目标力值 → 比对当前步与上一步，保留更接近者
            error_prev = abs(self.target_force - self.prev_fz)
            error_curr = abs(self.target_force - self.current_fz)
            self.get_logger().info(
                f'跨越目标 {self.target_force:.1f}N | '
                f'prev: Fz={self.prev_fz:.2f}N err={error_prev:.2f}N @ Z={self.prev_z:.6f} | '
                f'curr: Fz={self.current_fz:.2f}N err={error_curr:.2f}N @ Z={self.current_z:.6f}'
            )

            if error_prev <= error_curr:
                # 上一步更接近 → 直接回退到上一步
                self.current_z = self.prev_z
                self.get_logger().info(
                    f'>>> 上一步更接近 (err={error_prev:.2f} ≤ {error_curr:.2f})，'
                    f'回退至 Z={self.current_z:.6f}，保压保持'
                )
                self._publish_cartepos()
                self.phase = 'HOLD'
            else:
                # 当前步更接近 → 精调一步尝试更好
                self.error_before = error_curr
                self.hold_z = self.current_z
                self.get_logger().info(
                    f'当前步更接近 (err={error_curr:.2f} < {error_prev:.2f})，下压一步精调...'
                )
                self._do_step(self.step_size, self.fine_stabilize_cycles)
                self.phase = 'FINE_TUNE_WAIT'
            return

        # 未到目标 → 记录当前值作为上一步，继续下压
        self.prev_fz = self.current_fz
        self.prev_z = self.current_z

        fine_threshold = self.target_force * self.fine_threshold_ratio
        if self.current_fz < fine_threshold:
            step = self.approach_step_size
            wait = self.coarse_stabilize_cycles
            tag = '粗调'
        else:
            step = self.step_size
            wait = self.fine_stabilize_cycles
            tag = '精细'
        self._do_step(step, wait, tag)

    # ==================== 精调（FINE_TUNE） ====================
    def _evaluate_fine_tune(self):
        error_after = abs(self.target_force - self.current_fz)
        self.get_logger().info(
            f'精调结果 | Fz={self.current_fz:.2f}N | '
            f'error_before={self.error_before:.2f}N → error_after={error_after:.2f}N'
        )

        if error_after <= self.error_before:
            # 更接近目标或不变 → 保持
            self.get_logger().info(
                f'>>> 精调完成，误差不增 (≤{self.error_before:.2f}N)，保压保持 Z={self.current_z:.6f}'
            )
            self.phase = 'HOLD'
        else:
            # 误差增大 → 回退
            self.current_z = self.hold_z
            self.get_logger().info(
                f'>>> 误差增大，回退至 Z={self.current_z:.6f}，保压保持'
            )
            self._publish_cartepos()
            self.phase = 'HOLD'

    # ==================== 纠偏恢复 ====================
    def _evaluate_recover(self):
        error_after = abs(self.target_force - self.current_fz)
        self.get_logger().info(
            f'纠偏结果 | Fz={self.current_fz:.2f}N | '
            f'err: {self.error_before:.2f}N → {error_after:.2f}N'
        )
        if error_after <= self.error_before:
            self.get_logger().info(
                f'>>> 纠偏有效，保压保持 Z={self.current_z:.6f}'
            )
            self.phase = 'HOLD'
        else:
            self.current_z = self.prev_z
            self.get_logger().info(
                f'>>> 纠偏无效，回退至 Z={self.current_z:.6f}，保压保持'
            )
            self._publish_cartepos()
            self.phase = 'HOLD'

    # ==================== 单步 ====================
    def _do_step(self, step, wait_cycles, tag='', direction='down'):
        if direction == 'down':
            self.current_z -= step
            self.total_descent += step
        else:
            self.current_z += step
        self.step_wait_counter = wait_cycles
        self._log_seq += 1
        if self._log_seq % 10 == 0 or tag:
            dir_label = '↓' if direction == 'down' else '↑'
            self.get_logger().info(
                f'[{tag or dir_label}] step={step*1e3:.2f}mm {dir_label} | Z={self.current_z:.6f} | '
                f'Fz={self.current_fz:.2f}N | wait={wait_cycles}cyc | '
                f'descent={self.total_descent*1e3:.1f}mm'
            )

    # ==================== 发布 Cartepos ====================
    def _publish_cartepos(self):
        msg = Cartepos()
        msg.pose.position.x = float(self.start_x)
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
