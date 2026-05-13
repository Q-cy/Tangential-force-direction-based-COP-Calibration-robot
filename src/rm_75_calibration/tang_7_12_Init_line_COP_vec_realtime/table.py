# file_name: table.py

import os
import csv
import numpy as np

# 定义CSV表头
CSV_HEADER = [
    "timestamp", "rel_ms",
    # ch1 ~ ch84
    "ch1", "ch2", "ch3", "ch4", "ch5", "ch6", "ch7",
    "ch8", "ch9", "ch10", "ch11", "ch12", "ch13", "ch14",
    "ch15", "ch16", "ch17", "ch18", "ch19", "ch20", "ch21",
    "ch22", "ch23", "ch24", "ch25", "ch26", "ch27", "ch28",
    "ch29", "ch30", "ch31", "ch32", "ch33", "ch34", "ch35",
    "ch36", "ch37", "ch38", "ch39", "ch40", "ch41", "ch42",
    "ch43", "ch44", "ch45", "ch46", "ch47", "ch48", "ch49",
    "ch50", "ch51", "ch52", "ch53", "ch54", "ch55", "ch56",
    "ch57", "ch58", "ch59", "ch60", "ch61", "ch62", "ch63",
    "ch64", "ch65", "ch66", "ch67", "ch68", "ch69", "ch70",
    "ch71", "ch72", "ch73", "ch74", "ch75", "ch76", "ch77",
    "ch78", "ch79", "ch80", "ch81", "ch82", "ch83", "ch84",
    # 力传感器数据
    "Fx", "Fy", "Fz", "Mx", "My", "Mz",
    # 时间戳相关
    "press_t", "force_t", "dt",
    # CoP 偏移分量
    "delta_CoP_X", "delta_CoP_Y",
    # Force 分量
    "delta_Force_X", "delta_Force_Y", "delta_Force_Z",
    # 角度和幅值
    "ADC_angle", "ADC_mag", "Force_angle", "Force_mag",
    # 标定后的切向力
    "Fx_cal", "Fy_cal", "Force_cal_mag", "Force_cal_angle",
    "valid"
]

def auto_get_csv_path(save_dir: str) -> str:
    os.makedirs(save_dir, exist_ok=True)
    idx = 1
    while os.path.exists(f"{save_dir}/data_{idx}.csv"):
        idx += 1
    return f"{save_dir}/data_{idx}.csv"

def init_csv_file(file_path: str) -> tuple:
    csv_file_obj = open(file_path, "w", encoding="utf-8", newline="")
    csv_writer = csv.writer(csv_file_obj)
    csv_writer.writerow(CSV_HEADER)
    print(f"📂 CSV文件已初始化：{file_path}")
    return csv_writer, csv_file_obj

def build_csv_row(
    press_timestamp: float,
    rel_ms: int,
    ch_data: list,
    force_data: list,
    force_timestamp: float,
    delta_cop_x: float,
    delta_cop_y: float,
    delta_force_x: float,
    delta_force_y: float,
    delta_force_z: float,
    adc_angle: float,
    adc_mag: float,
    force_angle: float,
    force_mag: float,
    fx_cal: float = None,
    fy_cal: float = None,
    force_cal_mag: float = None,
    force_cal_angle: float = None,
    valid: int = 0,
) -> list:
    dt = abs(press_timestamp - force_timestamp)
    csv_row = [
        press_timestamp * 1000,
        rel_ms,
        *ch_data,
        *force_data,
        press_timestamp,
        force_timestamp,
        dt,
        delta_cop_x,
        delta_cop_y,
        delta_force_x,
        delta_force_y,
        delta_force_z,
        adc_angle,
        adc_mag,
        force_angle,
        force_mag,
        fx_cal if fx_cal is not None else float('nan'),
        fy_cal if fy_cal is not None else float('nan'),
        force_cal_mag if force_cal_mag is not None else float('nan'),
        force_cal_angle if force_cal_angle is not None else float('nan'),
        valid,
    ]
    return csv_row
