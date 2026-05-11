#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# 导入睿尔曼(RM)机械臂自定义的消息类型
from rm_ros_interfaces.msg import Movejp, Movel

class ArmControllerNode(Node):
    def __init__(self):
        super().__init__('arm_controller_node')
        
        # 创建发布者
        self.pub_movejp = self.create_publisher(Movejp, '/rm_driver/movej_p_cmd', 10)
        self.pub_movel = self.create_publisher(Movel, '/rm_driver/movel_cmd', 10)
        
        # 稍微等待发布者与底层驱动建立连接
        time.sleep(1.0)

        # ================= 核心参数配置区 =================
        # 初始位姿参数，单位m
        self.init_x = 89.5 / 1000.0
        self.init_y = 397.2 / 1000.0
        self.init_z = (56.15 + 1) / 1000.0
        # self.init_x = 0.4
        # self.init_y = 0.0
        # self.init_z = 0.06
        
        self.ori_x = 0.0
        self.ori_y = 1.0
        self.ori_z = 0.0
        self.ori_w = 0.0

        self.speed = 10

        # 运动逻辑参数
        # 观察你的指令
        self.z_base_press = 0.001  # 基础下压距离 z，单位 m (初始下压1mm)
        self.m_step = 0.0005       # 每次外循环增加的距离 m，最小增量0.05mm
        self.n_sub_loops = 10      # 子循环次数 n
        self.outer_loops = 1      # 外循环总次数 (0, 1, 2, 3, 4)
        
        self.wait_time_hold = 5      # 每次下压保持的物理时间(秒)，请根据实际运动幅度调整
        self.wait_time_reset = 5     # 每次复位等待的物理时间(秒)，请根据实际运动幅度调整
        # ==================================================

    def get_movejp_msg(self, x, y, z):
        """构造 Movejp 消息"""
        msg = Movejp()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(self.ori_x)
        msg.pose.orientation.y = float(self.ori_y)
        msg.pose.orientation.z = float(self.ori_z)
        msg.pose.orientation.w = float(self.ori_w)
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        return msg

    def get_movel_msg(self, x, y, z):
        """构造 Movel 消息"""
        msg = Movel()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.x = float(self.ori_x)
        msg.pose.orientation.y = float(self.ori_y)
        msg.pose.orientation.z = float(self.ori_z)
        msg.pose.orientation.w = float(self.ori_w)
        msg.speed = self.speed
        msg.trajectory_connect = 0
        msg.block = True
        return msg

    def run_process(self):
        """执行用户要求的运动流程"""
        
        # 1. 初始化：使用movejp指令回到初始位置
        self.get_logger().info("1. 初始化：移动到初始位置...")
        init_msg = self.get_movejp_msg(self.init_x, self.init_y, self.init_z)
        self.pub_movejp.publish(init_msg)
        
        # 等待5秒
        self.get_logger().info("等待5秒...")
        time.sleep(5.0)

        # 外层循环：控制下压深度增量 (循环0, 循环1, 循环2...)
        for i in range(self.outer_loops):
            # 计算当前循环的下压深度：Z = 初始Z - 基础下压距离(z) + i * 增量(m)
            current_press_dist = self.z_base_press + (i * self.m_step)
            target_z = self.init_z - current_press_dist
            
            self.get_logger().info(f"\n========== 开始外循环 {i} | 下压深度 delta Z: {current_press_dist:.6f} ==========")

            # 内层子循环 n 遍
            for j in range(self.n_sub_loops):
                self.get_logger().info(f"  [子循环 {j+1}/{self.n_sub_loops}] 下压...")
                
                # 发送下压指令
                press_msg = self.get_movel_msg(self.init_x, self.init_y, target_z)
                self.pub_movel.publish(press_msg)
                time.sleep(self.wait_time_hold) # 等待机械臂运动完成
                
                self.get_logger().info(f"  [子循环 {j+1}/{self.n_sub_loops}] 复位...")
                
                # 发送回退指令 (回到初始Z高度)
                retract_msg = self.get_movel_msg(self.init_x, self.init_y, self.init_z)
                self.pub_movel.publish(retract_msg)
                time.sleep(self.wait_time_reset) # 等待机械臂运动完成
                
        self.get_logger().info("所有运动流程执行完毕！")


def main(args=None):
    rclpy.init(args=args)
    
    node = ArmControllerNode()
    
    try:
        # 执行流程 (因为是顺序执行任务，不需要 rclpy.spin())
        node.run_process()
    except KeyboardInterrupt:
        node.get_logger().info("程序被用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()