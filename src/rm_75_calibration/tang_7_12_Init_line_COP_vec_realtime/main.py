# file_name: main.py

import time
import os
from collections import deque
import numpy as np
import threading
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

# 导入自定义模块
import angle as angle
import COP as COP
import data as data
import realtime as realtime
import table as table
import calibrate

# ===================== 配置 =====================
SAVE_DIR = "/home/qcy/Project/data/2.PZT_tangential/weight/test"
TARGET_FPS = 100
MAX_TIME_DIFF = 0.015
stop_event = threading.Event()
plot = None

# ===================== ROS2 力数据订阅节点 =====================
class ForceDataSubscriber(Node):
    """从 /force_sensor_data 话题订阅六维力数据，写入 TimestampedBuffer"""
    def __init__(self, buffer):
        super().__init__('force_data_subscriber')
        self.buffer = buffer
        self.sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._callback, 10
        )

    def _callback(self, msg):
        ts = time.perf_counter()
        data = [
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z,
            msg.wrench.torque.x,
            msg.wrench.torque.y,
            msg.wrench.torque.z,
        ]
        self.buffer.append({"t": ts, "data": data})

# ===================== 控制状态订阅节点 =====================
class PhaseSubscriber(Node):
    def __init__(self):
        super().__init__('phase_subscriber')
        self.latest_phase = ''
        self.lock = threading.Lock()
        self.sub = self.create_subscription(
            String, '/force_control_state', self._callback, 10
        )

    def _callback(self, msg):
        with self.lock:
            self.latest_phase = msg.data

    def is_valid(self):
        with self.lock:
            p = self.latest_phase
        return 'HOLD' in p or 'STEP_DWELL' in p

# ===================== 采集线程 =====================
class PressureThread(threading.Thread):
    def __init__(self, sensor, buf):
        super().__init__(daemon=True)
        self.s = sensor
        self.buf = buf
    def run(self):
        while not stop_event.is_set():
            ts = time.perf_counter()
            raw = self.s.read_data()
            if raw:
                try:
                    d = self.s.decode(raw)
                    self.buf.append({"t":ts,"data":d})
                except:
                    pass
            time.sleep(0.001)

# ===================== ROS2 spin 线程 =====================
def ros2_spin(executor):
    while not stop_event.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.01)

# ===================== 数据循环 =====================
def data_loop(force_node, phase_node=None):
    global plot
    # 自动获取CSV文件路径
    csv_path = table.auto_get_csv_path(SAVE_DIR)
    # 初始化CSV文件（写入表头）
    csv_writer, csv_file_obj = table.init_csv_file(csv_path)

    # 初始化传感器（只读压力传感器，力传感器数据来自 ROS2 话题）
    s_press = data.PressureSensor()
    print("✅ 压力传感器初始化完成")
    print("📡 六维力数据来自 ROS2 /force_sensor_data 话题")

    # 初始化缓存
    buf_press = data.TimestampedBuffer(500)
    buf_force = force_node.buffer

    # 启动压力采集线程
    t1 = PressureThread(s_press, buf_press)
    t1.start()

    print("🎨 绘图已打开")
    t0 = time.perf_counter()

    # 尝试加载标定插值器
    cal_path = os.path.join(SAVE_DIR, "cal_lookup.npz")
    cal_ready = False
    points, fx_vals, fy_vals = None, None, None
    if os.path.exists(cal_path):
        try:
            points, fx_vals, fy_vals = calibrate.load_lookup(cal_path)
            cal_ready = True
            print(f"📐 标定查找表已加载: {cal_path}")
        except Exception as e:
            print(f"⚠️ 标定查找表加载失败: {e}")
    else:
        print("💡 未找到标定文件 cal_lookup.npz。如需标定：")
        print("   python calibrate.py <N>")

    # 中值滤波窗口（窗口大小=5）
    MEDIAN_WINDOW = 5
    buf_dx = deque(maxlen=MEDIAN_WINDOW)
    buf_dy = deque(maxlen=MEDIAN_WINDOW)
    buf_fx = deque(maxlen=MEDIAN_WINDOW)
    buf_fy = deque(maxlen=MEDIAN_WINDOW)
    buf_fz = deque(maxlen=MEDIAN_WINDOW)

    while not stop_event.is_set():
        now = time.perf_counter()
        rel_ms = int((now - t0) * 1000)  # 相对毫秒数

        # 获取最新压力传感器数据
        press_data_item = buf_press.get_latest()
        if not press_data_item:
            time.sleep(0.001)
            continue

        # 匹配最近的力传感器数据
        force_data_item = buf_force.find_closest(press_data_item["t"])
        if not force_data_item or abs(press_data_item["t"] - force_data_item["t"]) > MAX_TIME_DIFF:
            time.sleep(0.001)
            continue

        # 计算CoP和方向数据
        base = COP.subtract_baseline(press_data_item["data"])
        cop_res = COP.compute_pressure_direction(base)
        cx, cy = cop_res[0], cop_res[1]
        dx, dy = cop_res[6], cop_res[7] # delta_cop_x, delta_cop_y
        bx, by = cop_res[8], cop_res[9]

        # 解析力传感器数据
        fx, fy, fz, mx, my, mz = force_data_item["data"]

        # 中值滤波：消除偶发尖峰
        buf_dx.append(dx)
        buf_dy.append(dy)
        buf_fx.append(fx)
        buf_fy.append(fy)
        buf_fz.append(fz)
        dx_f = np.median(buf_dx)
        dy_f = np.median(buf_dy)
        fx_f = np.median(buf_fx)
        fy_f = np.median(buf_fy)
        fz_f = np.median(buf_fz)
        total_pressure = np.sum(press_data_item["data"])

        # 计算角度和幅值（使用滤波后的值）
        adc_angle, adc_mag = angle.compute_PZT_angle(dx_f, dy_f)
        force_angle, force_mag = angle.compute_6Dforce_angle(fx_f, fy_f)

        # 标定：CoP位移 → 切向力（插值）
        if cal_ready:
            fx_cal, fy_cal = calibrate.apply(dx_f, dy_f, points, fx_vals, fy_vals)
            cal_angle, cal_mag = angle.compute_vector_angle(fx_cal, fy_cal)
        else:
            fx_cal, fy_cal, cal_angle, cal_mag = None, None, None, None

        valid = 1 if (phase_node and phase_node.is_valid()) else 0

        # 构造CSV行数据（调用封装函数）
        csv_row = table.build_csv_row(
            press_timestamp=press_data_item["t"],
            rel_ms=rel_ms,
            ch_data=press_data_item["data"],
            force_data=force_data_item["data"],
            force_timestamp=force_data_item["t"],
            delta_cop_x=dx_f,
            delta_cop_y=dy_f,
            delta_force_x=fx_f,
            delta_force_y=fy_f,
            delta_force_z=fz_f,
            adc_angle=adc_angle,
            adc_mag=adc_mag,
            force_angle=force_angle,
            force_mag=force_mag,
            fx_cal=fx_cal,
            fy_cal=fy_cal,
            force_cal_mag=cal_mag,
            force_cal_angle=cal_angle,
            valid=valid,
        )

        # 写入CSV行
        csv_writer.writerow(csv_row)
        csv_file_obj.flush()  # 立即刷新到文件

        # 更新绘图数据
        plot.set_data(
            adc_angle, adc_mag, force_angle, force_mag,
            base, total_pressure, force_mag,
            cx, cy, bx, by, dx_f, dy_f,
            fx_f, fy_f, fz_f,
            fx_cal, fy_cal, cal_angle, cal_mag,
        )
        # 追加全程数据
        if COP.contact_initialized:
                    plot.append_full_data(rel_ms,
                                          adc_angle, adc_mag, total_pressure, dx_f, dy_f,
                                          force_angle, force_mag, fz_f, fx_f, fy_f,
                                          cal_angle, cal_mag, fx_cal, fy_cal)

        # 控制采集频率
        elapsed = time.perf_counter() - now
        time.sleep(max(0, 1/TARGET_FPS - elapsed))

    # 关闭CSV文件
    csv_file_obj.close()
    print("✅ CSV文件已关闭")

# ===================== 主函数 =====================
def main():
    global plot

    # 初始化 ROS2
    rclpy.init(args=None)

    # 创建力数据订阅节点
    buf_force = data.TimestampedBuffer(500)
    force_node = ForceDataSubscriber(buf_force)
    phase_node = PhaseSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(force_node)
    executor.add_node(phase_node)

    # 启动 ROS2 spin 线程
    spin_thread = threading.Thread(target=ros2_spin, args=(executor,), daemon=True)
    spin_thread.start()

    plot = realtime.RealTimePlot()
    # 启动数据采集线程
    data_thread = threading.Thread(target=data_loop, args=(force_node, phase_node))
    data_thread.start()
    plt.show()  # 阻塞直到关闭绘图窗口

    # 停止采集线程
    stop_event.set()
    data_thread.join(timeout=2)
    rclpy.shutdown()
    # 绘制全程静态图
    plot.plot_full_magnitude_curve(SAVE_DIR)

if __name__ == "__main__":
    main()
