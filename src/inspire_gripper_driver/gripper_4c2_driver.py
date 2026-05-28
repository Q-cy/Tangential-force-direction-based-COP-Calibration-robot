import minimalmodbus
import serial
import threading
import time

class EG2Gripper:
    """
    北京因时机器人 EG2-4X2 伺服电动夹爪驱动 (Modbus RTU)
    基于用户手册版本 V1.0.7
    """

    # 寄存器地址定义 (十进制，对应手册第3章表格)
    REG_SAVE_PARAM      = 1
    REG_RESET_DEFAULT   = 2
    REG_SLAVE_ID        = 3
    REG_BAUD_RATE       = 4
    REG_CATCH_MODE      = 5
    REG_STOP            = 6
    REG_FAULT_CLEAR     = 7
    
    REG_SET_OPEN_LEN    = 10  # (0x10) 设置开口度
    REG_SET_SPEED       = 11  # (0x11) 设置速度
    REG_SET_FORCE       = 12  # (0x12) 设置夹持力
    
    REG_MAX_OPEN_LIMIT  = 16  # (0x16) 最大开口限制
    REG_MIN_OPEN_LIMIT  = 17  # (0x17) 最小开口限制
    
    # 只读寄存器
    REG_ACTUAL_OPEN_LEN = 61  # (0x61) 实际开口度
    REG_CURRENT         = 62  # (0x62) 电流
    REG_TEMP            = 63  # (0x63) 温度
    REG_ERROR_CODE      = 64 # (0x64) 故障码
    REG_STATUS_CODE     = 65 # (0x65) 状态码

    # 状态码定义
    STATUS_DESC = {
        1: "张开到位且停止",
        2: "闭合到位且停止",
        3: "停止且空闲",
        4: "正在夹取(闭合)",
        5: "正在张开",
        6: "夹取中遇到物体停止(力控保持)"
    }

    def __init__(self, port, slave_address=1, baudrate=115200, debug=False):
        """
        初始化夹爪连接
        :param port: 串口号 (例如 'COM3' 或 '/dev/ttyUSB0')
        :param slave_address: Modbus 从站 ID (默认 1)
        :param baudrate: 波特率 (默认 115200)
        :param debug: 是否打印调试信息
        """
        self.lock = threading.Lock()
        
        try:
            self.instrument = minimalmodbus.Instrument(port, slave_address)
            self.instrument.serial.baudrate = baudrate
            self.instrument.serial.bytesize = 8
            self.instrument.serial.parity   = serial.PARITY_NONE
            self.instrument.serial.stopbits = 1
            self.instrument.serial.timeout  = 0.5 # 秒
            self.instrument.mode = minimalmodbus.MODE_RTU
            self.instrument.clear_buffers_before_each_transaction = True
            self.instrument.debug = debug
        except Exception as e:
            print(f"Error initializing serial port: {e}")
            raise

    def _write_register(self, register, value, decimals=0):
        """线程安全的写寄存器封装"""
        with self.lock:
            try:
                self.instrument.write_register(register, value, decimals)
            except IOError as e:
                print(f"Write Error on Reg {register}: {e}")
                raise

    def _read_register(self, register, decimals=0):
        """线程安全的读寄存器封装"""
        with self.lock:
            try:
                return self.instrument.read_register(register, decimals)
            except IOError as e:
                print(f"Read Error on Reg {register}: {e}")
                raise

    def _read_registers(self, start_register, number_of_registers):
        """线程安全的批量读寄存器"""
        with self.lock:
            try:
                return self.instrument.read_registers(start_register, number_of_registers)
            except IOError as e:
                print(f"Batch Read Error: {e}")
                raise

    # ================= 用户控制 API =================

    def set_motion_params(self, speed=500, force=500):
        """
        设置运动参数 (速度和力度)
        注意：手册说明“设置此寄存器后夹爪不会动作”，仅更新参数。
        :param speed: 10-1000 (无量纲)
        :param force: 100-1000 (对应 100g - 1000g/2000g)
        """
        # 参数范围检查
        speed = max(10, min(1000, int(speed)))
        force = max(100, min(1000, int(force)))
        
        # 使用批量写入功能码 0x10 (Write Multiple Registers)
        # 根据手册举例3，可以一次性写入，但为了简单稳健，这里分两次或批量
        with self.lock:
            # 对应地址 17 和 18
            self.instrument.write_registers(self.REG_SET_SPEED, [speed, force])
        print(f"Params Set -> Speed: {speed}, Force: {force}")

    def move_to(self, position):
        """
        控制夹爪移动到指定开口度
        :param position: 0-1000 (0=最小开口, 1000=最大开口)
        """
        position = max(0, min(1000, int(position)))
        self._write_register(self.REG_SET_OPEN_LEN, position)
        # print(f"Command: Move to {position}")

    def stop(self):
        """急停"""
        self._write_register(self.REG_STOP, 1)
        print("Command: Emergency Stop")

    def clear_fault(self):
        """清除故障 (Fault Reset)"""
        self._write_register(self.REG_FAULT_CLEAR, 1)
        print("Command: Clear Faults")

    def set_catch_mode(self, mode):
        """
        设置夹取模式
        :param mode: 0=一次夹取, 1=持续夹取(掉力补夹)
        """
        if mode not in [0, 1]:
            raise ValueError("Mode must be 0 or 1")
        self._write_register(self.REG_CATCH_MODE, mode)

    def get_status_full(self):
        """
        获取夹爪完整状态
        根据手册举例1，批量读取地址 61-65 (0x3D)
        :return: dict 包含 位置, 电流, 温度, 故障, 状态
        """
        # 手册中地址：61(位置), 62(电流), 63(温度), 64(故障码), 65(状态码)
        # 注意：minimalmodbus 使用十进制地址
        try:
            # 读取 5 个寄存器：61, 62, 63, 64, 65
            data = self._read_registers(self.REG_ACTUAL_OPEN_LEN, 5)
            
            pos = data[0]
            current = data[1]
            # 电流可能是补码形式(虽然手册写0-2000，但在举例1中出现了 FF FE = -2，需要处理有符号数)
            if current > 32767:
                current = current - 65536
                
            temp = data[2]
            error_code = data[3]
            status_code = data[4]

            status_str = self.STATUS_DESC.get(status_code, "未知状态")
            errors = self._parse_error_code(error_code)

            return {
                "position": pos,
                "current_mA": current,
                "temperature_C": temp,
                "error_code": error_code,
                "error_msg": errors,
                "status_code": status_code,
                "status_msg": status_str
            }
        except Exception as e:
            print(f"Failed to get status: {e}")
            return None

    def _parse_error_code(self, code):
        """解析故障码位"""
        errors = []
        if code & (1 << 0): errors.append("堵转故障")
        if code & (1 << 1): errors.append("过温故障")
        if code & (1 << 2): errors.append("过流故障")
        if code & (1 << 3): errors.append("驱动器运行故障")
        if code & (1 << 4): errors.append("内部通信故障")
        return errors if errors else ["正常"]

    def save_parameters(self):
        """保存参数到内部 Flash (掉电不丢失)"""
        self._write_register(self.REG_SAVE_PARAM, 1)
        print("Parameters Saved to Flash")

# ================= 测试代码 =================
if __name__ == "__main__":
    
    PORT_NAME = '/dev/port03' 
    
    try:
        # 初始化夹爪 (默认ID为1)
        gripper = EG2Gripper(port=PORT_NAME, slave_address=1)
        
        print("连接成功，读取初始状态...")
        status = gripper.get_status_full()
        print(status)

        # 1. 清除可能存在的故障
        if status['error_code'] > 0:
            gripper.clear_fault()
            time.sleep(0.5)

        # 2. 设置速度和力度
        # 速度: 500 (中速), 力度: 300 (300g)
        gripper.set_motion_params(speed=500, force=300)

        # 3. 完全张开 (1000)
        print("正在张开...")
        gripper.move_to(1000)
        
        # 轮询直到到位
        for _ in range(20):
            time.sleep(0.2)
            s = gripper.get_status_full()
            # 状态 1 = 张开到位
            if s['status_code'] == 1:
                print("已张开到位")
                break
        
        time.sleep(1)

        # 4. 闭合 (0)
        print("正在闭合...")
        gripper.move_to(0)
        
        # 轮询直到到位
        for _ in range(20):
            time.sleep(0.2)
            s = gripper.get_status_full()
            # 状态 2 = 闭合到位, 状态 6 = 夹到物体
            if s['status_code'] in [2, 6]:
                print(f"动作结束: {s['status_msg']}")
                break

    except Exception as e:
        print(f"程序运行出错: {e}")