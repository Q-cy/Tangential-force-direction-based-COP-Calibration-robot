#!/usr/bin/env python3
"""
六维力传感器 ROS2 驱动节点
- 启动后自动零点校准（去皮）
- 200Hz 发布六维力数据话题 /force_sensor_data (WrenchStamped)
- 独立传感器读取线程，最大速率采集
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
import threading
import time

from rm_75_calibration.data import SixAxisForceSensor


class ForceSensorNode(Node):
    def __init__(self):
        super().__init__('force_sensor_node')

        # ---- 传感器初始化 ----
        self.sensor = SixAxisForceSensor()
        self.latest_data = None
        self.data_lock = threading.Lock()
        self.sensor_running = False
        self.sensor_thread = None

        # ---- 连续读取失败计数（用于自动重连） ----
        self.fail_count = 0
        self.max_fail = 50

        # ---- 发布者 ----
        self.force_pub = self.create_publisher(
            WrenchStamped, '/force_sensor_data', 10
        )

        # ---- 200Hz 发布定时器 ----
        self.pub_timer = self.create_timer(0.005, self._publish_callback)

        # ---- 启动流程：先校准，再开启传感器线程 ----
        self.get_logger().info('六维力传感器驱动节点启动')
        self._calibrate()
        self._start_sensor_thread()

    # ==================== 零点校准 ====================
    def _calibrate(self):
        self.get_logger().info('开始零点校准（去皮），请确保传感器空载...')
        self.sensor.calibrate_zero()
        self.get_logger().info(
            f'校准完成，零点: '
            f'Fx={self.sensor.zero_data[0]:.3f} '
            f'Fy={self.sensor.zero_data[1]:.3f} '
            f'Fz={self.sensor.zero_data[2]:.3f} '
            f'Mx={self.sensor.zero_data[3]:.3f} '
            f'My={self.sensor.zero_data[4]:.3f} '
            f'Mz={self.sensor.zero_data[5]:.3f}'
        )
        self.get_logger().info(
            '零点校准完成，开始发布 /force_sensor_data (200Hz)'
        )

    # ==================== 传感器读取线程 ====================
    def _start_sensor_thread(self):
        self.sensor_running = True
        self.sensor_thread = threading.Thread(target=self._sensor_loop, daemon=True)
        self.sensor_thread.start()

    def _sensor_loop(self):
        while self.sensor_running and rclpy.ok():
            data = self.sensor.read()
            if data is not None:
                with self.data_lock:
                    self.latest_data = data
                self.fail_count = 0
            else:
                self.fail_count += 1
                if self.fail_count >= self.max_fail:
                    self.get_logger().warn(
                        f'传感器连续 {self.fail_count} 次读取失败，尝试重连...'
                    )
                    self.sensor.reconnect()
                    self.fail_count = 0
                    time.sleep(0.5)

    # ==================== 200Hz 发布回调 ====================
    def _publish_callback(self):
        with self.data_lock:
            data = self.latest_data
        if data is None:
            return

        Fx, Fy, Fz, Mx, My, Mz = data

        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'force_sensor_link'
        msg.wrench.force.x = float(Fx)
        msg.wrench.force.y = float(Fy)
        msg.wrench.force.z = float(Fz)
        msg.wrench.torque.x = float(Mx)
        msg.wrench.torque.y = float(My)
        msg.wrench.torque.z = float(Mz)
        self.force_pub.publish(msg)

    # ==================== 清理 ====================
    def destroy_node(self):
        self.sensor_running = False
        if self.sensor_thread and self.sensor_thread.is_alive():
            self.sensor_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ForceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
