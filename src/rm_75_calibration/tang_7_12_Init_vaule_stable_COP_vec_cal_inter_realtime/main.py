# file_name: main.py
#
# 适配修改（相比原始版本）：
# - 新增 ForceDataSubscriber：订阅 /force_sensor_data 获取六维力数据
# - 新增 PhaseSubscriber：订阅 /force_control_state 获取力控状态
# - 新增 CopTriggerSubscriber：订阅 /cop_trigger 触发 COP 精修
# - 力数据来源从本地串口改为 ROS2 话题

import time
import os
from collections import deque
import numpy as np
import threading
from pyqtgraph.Qt import QtWidgets
import sys

import angle as angle
import COP as COP
import data as data
import table as table
import calibrate
import importlib

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import String

# ===================== ROS2 节点 =====================
class ForceDataSubscriber(Node):
    def __init__(self, buffer):
        super().__init__('force_data_subscriber')
        self.buffer = buffer
        self.sub = self.create_subscription(
            WrenchStamped, '/force_sensor_data', self._callback, 10
        )
    def _callback(self, msg):
        ts = time.perf_counter()
        self.buffer.append({"t": ts, "data": [
            msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z,
            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z,
        ]})

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

class CopTriggerSubscriber(Node):
    def __init__(self):
        super().__init__('cop_trigger_subscriber')
        self.sub = self.create_subscription(
            String, '/cop_trigger', self._callback, 10
        )
    def _callback(self, msg):
        if msg.data == 'refine':
            COP.trigger_cop_refine()
            print('[COP] trigger_cop_refine() called from force_control_node')

def ros2_spin(executor):
    while not g_main_stop_flag.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.01)

# ===================== 配置 =====================
MAIN_REALTIME_MODULE = "realtime"           # "realtime"=全显示, "realtime2"=仅压阻
MAIN_SAVE_DIR = "/home/qcy/Project/data/2.PZT_tangential/weight/test"  # 数据保存根目录
MAIN_CAL_MODE = "lookup"                       # "lookup"=纯查表, "fit"=纯拟合, "auto"=优先拟合回退查表
CAL_DIM = "2D"                                 # "2D"=CoP位移→Fx,Fy; "3D"=CoP位移+总压力→Fz,Fx,Fy

realtime = importlib.import_module(MAIN_REALTIME_MODULE)
MAIN_TARGET_FPS = 100                      # 目标采集帧率
MAIN_MAX_TIME_DIFF_S = 0.015               # 压力-力传感器最大时间匹配差(秒)
g_main_stop_flag = threading.Event()       # 全局停止信号
g_main_plot = None                         # 绘图对象引用

# ===================== 采集线程 =====================
class PressureThread(threading.Thread):                   
    def __init__(self, sensor, buf):                      
        super().__init__(daemon=True)
        self.s = sensor                                   
        self.buf = buf
    def run(self):
        while not g_main_stop_flag.is_set():
            ts = time.perf_counter()
            raw = self.s.read_data()
            if raw:
                try:
                    d = self.s.decode(raw)
                    self.buf.append({"t":ts,"data":d})
                except:
                    pass
            time.sleep(0.001)

class ForceThread(threading.Thread):
    def __init__(self, sensor, buf):
        super().__init__(daemon=True)
        self.s = sensor
        self.buf = buf
    def run(self):
        while not g_main_stop_flag.is_set():
            ts = time.perf_counter()
            d = self.s.read()
            if d:
                self.buf.append({"t":ts,"data":d})
            time.sleep(0.001)

# ===================== 数据循环 =====================
def data_loop(force_node, phase_node=None):
    global g_main_plot
    csv_path = table.auto_get_csv_path(MAIN_SAVE_DIR)
    csv_writer, csv_file_obj = table.init_csv_file(csv_path)

    has_press = True

    try:
        sensor_press = data.PressureSensor()
        buf_press = data.TimestampedBuffer(500)
        thread_press = PressureThread(sensor_press, buf_press)
        thread_press.start()
        print("✅ 压力传感器就绪")
    except Exception as e:
        has_press = False
        buf_press = None
        print(f"⚠️ 压力传感器未连接: {e}")

    buf_force = force_node.buffer
    has_force = True
    print("📡 六维力数据来自 ROS2 /force_sensor_data 话题")

    if not has_press:
        print("❌ 压力传感器未连接，退出")
        return

    print("🎨 绘图已打开")
    start_time_s = time.perf_counter()

    # 加载标定模型（查找表 + 拟合）
    cal_bin_path = os.path.join(MAIN_SAVE_DIR, "cal_lookup.bin")
    cal_fit_path = os.path.join(MAIN_SAVE_DIR, "cal_fit.bin")
    cal_lut_ready_flag = False
    cal_fit_ready_flag = False
    cal_pts_arr = cal_fx_arr = cal_fy_arr = None
    cal_coef_fx = cal_coef_fy = None
    if os.path.exists(cal_bin_path):
        try:
            cal_pts_arr, cal_fx_arr, cal_fy_arr = calibrate.load_lookup(cal_bin_path, dim=CAL_DIM)
            cal_lut_ready_flag = True
            print(f"📐 查找表已加载: {cal_bin_path}")
        except Exception as e:
            print(f"⚠️ 查找表加载失败: {e}")
    if os.path.exists(cal_fit_path):
        try:
            cal_coef_fx, cal_coef_fy = calibrate.load_fit_model(cal_fit_path, dim=CAL_DIM)
            cal_fit_ready_flag = True
            print(f"📐 拟合模型已加载: {cal_fit_path}")
        except Exception as e:
            print(f"⚠️ 拟合模型加载失败: {e}")
    if not cal_lut_ready_flag and not cal_fit_ready_flag:
        print("💡 未找到标定文件")

    median_filt_window = 5
    buf_cop_delta_x = deque(maxlen=median_filt_window)
    buf_cop_delta_y = deque(maxlen=median_filt_window)
    buf_force_fx = deque(maxlen=median_filt_window)
    buf_force_fy = deque(maxlen=median_filt_window)
    buf_force_fz = deque(maxlen=median_filt_window)

    _NAN6 = [float('nan')] * 6  # 力传感器占位

    while not g_main_stop_flag.is_set():
        loop_start_s = time.perf_counter()
        rel_time_ms = int((loop_start_s - start_time_s) * 1000)

        # ---- 采集压力数据 ----
        press_item = buf_press.get_latest() if has_press else None
        force_item = buf_force.get_latest() if has_force else None

        if press_item is None and force_item is None:
            time.sleep(0.001)
            continue

        # ---- 计算 PZT / CoP ----
        if press_item is not None:
            cop_res = COP.compute_pressure_direction(press_item["data"])
            base_sub_arr = np.array(press_item["data"])
            cop_curr_x, cop_curr_y = cop_res[0], cop_res[1]
            cop_delta_x, cop_delta_y = cop_res[6], cop_res[7]
            cop_base_x, cop_base_y = cop_res[8], cop_res[9]
            cop_state = cop_res[10]
            total_press_val = np.sum(press_item["data"])

            buf_cop_delta_x.append(cop_delta_x)
            buf_cop_delta_y.append(cop_delta_y)
            cop_delta_x_filt = np.median(buf_cop_delta_x)
            cop_delta_y_filt = np.median(buf_cop_delta_y)
            pzt_angle_deg, pzt_mag_val = angle.compute_PZT_angle(cop_delta_x_filt, cop_delta_y_filt)
        else:
            base_sub_arr = np.zeros(84)
            cop_curr_x = cop_curr_y = cop_delta_x = cop_delta_y = cop_base_x = cop_base_y = float('nan')
            cop_delta_x_filt = cop_delta_y_filt = 0.0
            pzt_angle_deg = pzt_mag_val = 0.0
            total_press_val = 0.0
            cop_state = 0

        # ---- 计算 Force ----
        if force_item is not None:
            force_fx_val, force_fy_val, force_fz_val = force_item["data"][:3]
            buf_force_fx.append(force_fx_val)
            buf_force_fy.append(force_fy_val)
            buf_force_fz.append(force_fz_val)
            force_fx_filt = np.median(buf_force_fx)
            force_fy_filt = np.median(buf_force_fy)
            force_fz_filt = np.median(buf_force_fz)
            force_angle_deg, force_mag_val = angle.compute_6Dforce_angle(force_fx_filt, force_fy_filt)
            force_data_out = force_item["data"]
            force_ts_out = force_item["t"]
        else:
            force_fx_val = force_fy_val = force_fz_val = float('nan')
            force_fx_filt = force_fy_filt = force_fz_filt = float('nan')
            force_angle_deg = force_mag_val = float('nan')
            force_data_out = _NAN6
            force_ts_out = float('nan')

        # ---- 标定 ----
        if MAIN_CAL_MODE == "fit" and press_item is not None and cal_fit_ready_flag:
            cal_fx_val, cal_fy_val = calibrate.apply_fit((cop_delta_x_filt, cop_delta_y_filt), cal_coef_fx, dim=CAL_DIM)
            cal_angle_deg, cal_mag_val = angle.compute_vector_angle(cal_fx_val, cal_fy_val)
        elif MAIN_CAL_MODE == "lookup" and press_item is not None and cal_lut_ready_flag:
            cal_fx_val, cal_fy_val = calibrate.apply((cop_delta_x_filt, cop_delta_y_filt), cal_pts_arr, cal_fx_arr, cal_fy_arr)
            cal_angle_deg, cal_mag_val = angle.compute_vector_angle(cal_fx_val, cal_fy_val)
        elif MAIN_CAL_MODE == "auto" and press_item is not None:
            if cal_fit_ready_flag:
                cal_fx_val, cal_fy_val = calibrate.apply_fit((cop_delta_x_filt, cop_delta_y_filt), cal_coef_fx, dim=CAL_DIM)
            elif cal_lut_ready_flag:
                cal_fx_val, cal_fy_val = calibrate.apply((cop_delta_x_filt, cop_delta_y_filt), cal_pts_arr, cal_fx_arr, cal_fy_arr)
            else:
                cal_fx_val = cal_fy_val = cal_angle_deg = cal_mag_val = None
            if cal_fx_val is not None:
                cal_angle_deg, cal_mag_val = angle.compute_vector_angle(cal_fx_val, cal_fy_val)
        else:
            cal_fx_val = cal_fy_val = cal_angle_deg = cal_mag_val = None

        # ---- CSV ----
        press_ts = press_item["t"] if press_item is not None else float('nan')
        csv_row = table.build_csv_row(
            press_timestamp=press_ts,
            rel_ms=rel_time_ms,
            ch_data=press_item["data"] if press_item is not None else [0]*84,
            force_data=force_data_out,
            force_timestamp=force_ts_out,
            delta_cop_x=cop_delta_x_filt,
            delta_cop_y=cop_delta_y_filt,
            delta_force_x=force_fx_filt,
            delta_force_y=force_fy_filt,
            delta_force_z=force_fz_filt,
            adc_angle=pzt_angle_deg,
            adc_mag=pzt_mag_val,
            force_angle=force_angle_deg,
            force_mag=force_mag_val,
            fx_cal=cal_fx_val,
            fy_cal=cal_fy_val,
            force_cal_mag=cal_mag_val,
            force_cal_angle=cal_angle_deg,
            cop_state=int(phase_node.latest_phase) if phase_node.latest_phase.isdigit() else 0,
            adc_sum=total_press_val,
        )
        csv_writer.writerow(csv_row)

        # ---- 更新绘图 ----
        g_main_plot.set_data(
            pzt_angle_deg, pzt_mag_val, force_angle_deg, force_mag_val,
            base_sub_arr, total_press_val, force_mag_val,
            cop_curr_x, cop_curr_y, cop_base_x, cop_base_y,
            cop_delta_x_filt, cop_delta_y_filt,
            force_fx_filt, force_fy_filt, force_fz_filt,
            cal_fx_val, cal_fy_val, cal_angle_deg, cal_mag_val,
            cop_state=cop_state,
        )
        if COP.g_cop_contact_init_flag:
            g_main_plot.append_full_data(
                rel_time_ms,
                pzt_angle_deg, pzt_mag_val, total_press_val,
                cop_delta_x_filt, cop_delta_y_filt,
                force_angle_deg, force_mag_val,
                force_fz_filt, force_fx_filt, force_fy_filt,
                cal_angle_deg, cal_mag_val, cal_fx_val, cal_fy_val)

        elapsed = time.perf_counter() - loop_start_s
        time.sleep(max(0, 1/MAIN_TARGET_FPS - elapsed))

    csv_file_obj.close()
    row_count = sum(1 for _ in open(csv_path)) - 1
    if row_count <= 0:
        os.remove(csv_path)
        print("⚠️ 无有效数据，CSV 已删除")
    else:
        print(f"✅ CSV已关闭（{row_count} 行）")

# ===================== 主函数 =====================
def main():
    global g_main_plot

    rclpy.init(args=None)
    buf_force = data.TimestampedBuffer(500)
    force_node = ForceDataSubscriber(buf_force)
    phase_node = PhaseSubscriber()
    cop_trigger_node = CopTriggerSubscriber()
    executor = SingleThreadedExecutor()
    executor.add_node(force_node)
    executor.add_node(phase_node)
    executor.add_node(cop_trigger_node)
    spin_thread = threading.Thread(target=ros2_spin, args=(executor,), daemon=True)
    spin_thread.start()

    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)

    g_main_plot = realtime.RealTimePlot()
    data_thread = threading.Thread(target=data_loop, args=(force_node, phase_node))
    data_thread.start()

    app.exec()

    g_main_stop_flag.set()
    data_thread.join(timeout=2)

    # 通知 force_control_node 返回初始点
    return_pub = force_node.create_publisher(String, '/force_control_return_home', 10)
    return_pub.publish(String(data='return'))
    time.sleep(1)  # 等待消息发送

    rclpy.shutdown()
    g_main_plot.plot_full_magnitude_curve(MAIN_SAVE_DIR)

if __name__ == "__main__":
    main()

