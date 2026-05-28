#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16, Int8, Int16MultiArray
from sensor_msgs.msg import JointState

# 导入驱动类
# 确保 eg2_driver.py 在同一目录下，或者在 PYTHONPATH 中
# try:
#     from . import gripper_4c2_driver
# except ImportError:
#     try:
#         import gripper_4c2_driver
#     except:
#         print("错误: 找不到 gripper_4c2_driver.py，请确保驱动文件位置")
#         sys.exit(1)

from inspire_gripper_driver.gripper_4c2_driver import EG2Gripper

class EG2GripperNode(Node):
    def __init__(self):
        super().__init__('eg2_gripper_node')

        # === 参数声明 ===
        self.declare_parameter('port', '/dev/port03')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('poll_rate', 200.0) # Hz

        # 获取参数
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        slave_id = self.get_parameter('slave_id').get_parameter_value().integer_value
        poll_rate = self.get_parameter('poll_rate').get_parameter_value().double_value

        self.get_logger().info(f"正在连接夹爪: {port} (Baud: {baud}, ID: {slave_id})...")

        # === 初始化驱动 ===
        try:
            self.driver = EG2Gripper(port=port, slave_address=slave_id, baudrate=baud)
            # 初始化时清除一次故障
            self.driver.clear_fault()
            self.get_logger().info("夹爪连接成功，驱动已就绪。")
        except Exception as e:
            self.get_logger().fatal(f"夹爪连接失败: {e}")
            exit(1)

        # === 创建发布者 ===
        # 1. 发布完整状态 (JSON 字符串)，方便调试查看所有细节
        self.pub_status = self.create_publisher(String, '/gripper/status_json', 10)
        # 2. 发布标准关节状态 (归一化 0.0 - 1.0)
        self.pub_joints = self.create_publisher(JointState, '/gripper/joint_states', 10)

        # === 创建订阅者 ===
        # 1. 控制位置 (0-1000)
        self.create_subscription(Int16, '/gripper/cmd_position', self.cb_position, 10)
        # 2. 控制参数 [速度, 力度]
        self.create_subscription(Int16MultiArray, '/gripper/cmd_params', self.cb_params, 10)
        # 3. 控制模式 (0=普通, 1=持续)
        self.create_subscription(Int8, '/gripper/cmd_mode', self.cb_mode, 10)

        # === 定时器 (状态轮询) ===
        self.timer = self.create_timer(1.0 / poll_rate, self.timer_callback)

    def timer_callback(self):
        """定时读取夹爪状态并发布"""
        try:
            # 读取完整状态
            status = self.driver.get_status_full()
            
            if status:
                # 1. 发布 JSON 调试信息
                msg_str = String()
                msg_str.data = json.dumps(status, ensure_ascii=False)
                self.pub_status.publish(msg_str)

                # 2. 发布 JointState (将 0-1000 映射为 0.0-1.0)
                # 注意: 这里的单位是归一化的，如果知道具体行程(mm)，可在此处转换
                msg_joint = JointState()
                msg_joint.header.stamp = self.get_clock().now().to_msg()
                msg_joint.name = ['eg2_gripper']
                
                # 位置归一化 0.0 - 1.0
                norm_pos = status['position']
                msg_joint.position = [norm_pos]
                
                # 将电流映射为力 (粗略估计，电流越大 Effort 越大)
                msg_joint.effort = [float(status['current_mA'])]
                
                self.pub_joints.publish(msg_joint)

        except Exception as e:
            self.get_logger().error(f"读取状态失败: {e}")

    def cb_position(self, msg):
        """回调: 位置控制"""
        # ros2 topic pub --once /eg2_gripper/cmd_position std_msgs/msg/Int16 "{data: 1000}"
        # ros2 topic pub --once /eg2_gripper/cmd_position std_msgs/msg/Int16 "{data: 0}"
        target_pos = msg.data
        self.get_logger().info(f"收到位置指令: {target_pos}")
        try:
            self.driver.move_to(target_pos)
        except Exception as e:
            self.get_logger().error(f"执行移动失败: {e}")

    def cb_params(self, msg):
        """回调: 参数设置 [速度, 力度]"""
        # ros2 topic pub --once /eg2_gripper/cmd_params std_msgs/msg/Int16MultiArray "{data: [500, 300]}"
        if len(msg.data) < 2:
            self.get_logger().warn("参数指令格式错误，需要 [speed, force]")
            return
        
        speed = msg.data[0]
        force = msg.data[1]
        self.get_logger().info(f"设置参数 -> 速度: {speed}, 力度: {force}")
        
        try:
            self.driver.set_motion_params(speed=speed, force=force)
        except Exception as e:
            self.get_logger().error(f"设置参数失败: {e}")

    def cb_mode(self, msg):
        """回调: 模式设置"""
        mode = msg.data
        mode_str = "持续夹取(掉力补夹)" if mode == 1 else "普通一次夹取"
        self.get_logger().info(f"设置模式 -> {mode_str}")
        
        try:
            self.driver.set_catch_mode(mode)
        except Exception as e:
            self.get_logger().error(f"设置模式失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = EG2GripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 关闭前尝试停止夹爪（可选）
        node.driver.stop() 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()