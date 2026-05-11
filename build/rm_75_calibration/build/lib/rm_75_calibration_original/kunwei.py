import serial
import serial.tools.list_ports
import time
import struct
import csv
import os

class SixAxisForceSensor:
    def __init__(self):
        self.ser = None
        self.auto_find_port()

    def auto_find_port(self):
        port = "/dev/ttyUSB6"
        try:
            print(f"尝试打开串口: {port}")
            s = serial.Serial(
                port=port,
                baudrate=460800,
                bytesize=8,
                stopbits=1,
                parity='N',
                timeout=0.01,
                rtscts=False,
                dsrdtr=False
            )
            self.ser = s
            print(f"成功打开六维力传感器串口: {port}")
            return
        except:
            print(f"打开串口 {port} 失败")

    def read_six_axis(self):
        if not self.ser:
            return None

        cmd = b'\x49\xAA\x0D\x0A'
        self.ser.reset_input_buffer()
        self.ser.write(cmd)
        resp = self.ser.read(28)

        if len(resp) != 28:
            return None
        if resp[0:2] != b'\x49\xAA':
            return None

        try:
            Fx = struct.unpack('<f', resp[2:6])[0]
            Fy = struct.unpack('<f', resp[6:10])[0]
            Fz = struct.unpack('<f', resp[10:14])[0]
            Mx = struct.unpack('<f', resp[14:18])[0]
            My = struct.unpack('<f', resp[18:22])[0]
            Mz = struct.unpack('<f', resp[22:26])[0]

            # # ✅ 全部乘以 9.8
            # Fx *= 9.8
            # Fy *= 9.8
            # Fz *= 9.8
            # Mx *= 9.8
            # My *= 9.8
            # Mz *= 9.8

            return [Fx, Fy, Fz, Mx, My, Mz]
        except:
            return None

# ==================== 主程序 ====================
if __name__ == '__main__':
    # save_dir = "/home/qcy/Project/data/PZT/robotic_arm"
    # os.makedirs(save_dir, exist_ok=True)

    # i = 1
    # while True:
    #     csv_path = os.path.join(save_dir, f"data_{i}.csv")
    #     if not os.path.exists(csv_path):
    #         break
    #     i += 1

    sensor = SixAxisForceSensor()
    # print(f"保存到: {csv_path}")
    print("终端1秒刷新一次\n")

    # with open(csv_path, "w", newline="", encoding="utf-8") as f:
    #     writer = csv.writer(f)
    #     writer.writerow(["timestamp", "rel_ms", "Fx", "Fy", "Fz", "Mx", "My", "Mz"])

    start_real = None
    next_collect = time.perf_counter()
    next_print = time.perf_counter()

    while True:
        while time.perf_counter() < next_collect:
            pass
        next_collect += 0.01

        data = sensor.read_six_axis()
        if not data:
            continue

        now = time.time()
        time_str = time.strftime("%Y%m%d%H%M%S", time.localtime(now))
        ms = int(now * 1000) % 1000
        timestamp = f"{time_str}{ms:03d}"

        if start_real is None:
            start_real = time.perf_counter()
            rel_ms = 0
        else:
            rel_ms = int((time.perf_counter() - start_real) * 1000)

        Fx, Fy, Fz, Mx, My, Mz = data
        # writer.writerow([timestamp, rel_ms, Fx, Fy, Fz, Mx, My, Mz])

        if time.perf_counter() >= next_print:
            next_print += 1.0
            print(f"{rel_ms:5d}ms | Fx={Fx:<7.4f} Fy={Fy:<7.4f} Fz={Fz:<7.4f} Mx={Mx:<7.4f} My={My:<7.4f} Mz={Mz:<7.4f}")
            print("\n")