"""
CoP 位移 → 切向力 标定模块（插值版）

使用 scipy.interpolate.LinearNDInterpolator 对配对数据 (dx, dy) → (Fx, Fy) 做分段线性插值。

用法:
  构建: python calibrate.py <csv_path>
  应用: from calibrate import load_interpolator, apply
"""

import os
import sys
import csv
import pickle
import numpy as np
from scipy.interpolate import LinearNDInterpolator


def build_from_csv(csv_path: str):
    """从 CSV 读取配对 (dx, dy, Fx, Fy) 数据，为 Fx 和 Fy 分别构建 LinearNDInterpolator。"""
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                dx = float(row["delta_CoP_X"])
                dy = float(row["delta_CoP_Y"])
                fx = float(row["delta_Force_X"])
                fy = float(row["delta_Force_Y"])
                rows.append((dx, dy, fx, fy))
            except (KeyError, ValueError):
                continue

    if len(rows) < 10:
        raise ValueError(f"有效数据点不足（{len(rows)} < 10），请检查CSV文件")

    data = np.array(rows)
    points = data[:, :2]
    fx_vals = data[:, 2]
    fy_vals = data[:, 3]

    interp_fx = LinearNDInterpolator(points, fx_vals)
    interp_fy = LinearNDInterpolator(points, fy_vals)

    print(f"\n{'='*50}")
    print(f"  标定插值器构建结果")
    print(f"{'='*50}")
    print(f"  数据点数: {len(data)}")
    print(f"  dx 范围: [{points[:,0].min():.4f}, {points[:,0].max():.4f}]")
    print(f"  dy 范围: [{points[:,1].min():.4f}, {points[:,1].max():.4f}]")
    print(f"  Fx 范围: [{fx_vals.min():.4f}, {fx_vals.max():.4f}] N")
    print(f"  Fy 范围: [{fy_vals.min():.4f}, {fy_vals.max():.4f}] N")

    fx_pred = interp_fx(points)
    fy_pred = interp_fy(points)
    valid_fx = ~np.isnan(fx_pred)
    valid_fy = ~np.isnan(fy_pred)
    if valid_fx.sum() > 0:
        rms_fx = np.sqrt(np.mean((fx_vals[valid_fx] - fx_pred[valid_fx]) ** 2))
        print(f"  Fx 自检 RMS: {rms_fx:.4f} N (有效点: {valid_fx.sum()})")
    if valid_fy.sum() > 0:
        rms_fy = np.sqrt(np.mean((fy_vals[valid_fy] - fy_pred[valid_fy]) ** 2))
        print(f"  Fy 自检 RMS: {rms_fy:.4f} N (有效点: {valid_fy.sum()})")
    print(f"{'='*50}\n")

    return interp_fx, interp_fy


def save_interpolator(interp_fx, interp_fy, path: str):
    """保存插值器到 pickle 文件"""
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with open(path, "wb") as f:
        pickle.dump({"interp_fx": interp_fx, "interp_fy": interp_fy}, f)
    print(f"  标定插值器已保存至: {path}")


def load_interpolator(path: str) -> tuple:
    """从 pickle 文件加载插值器，返回 (interp_fx, interp_fy)"""
    with open(path, "rb") as f:
        data = pickle.load(f)
    return data["interp_fx"], data["interp_fy"]


def apply(dx, dy, interp_fx, interp_fy) -> tuple:
    """对单点 (dx, dy) 插值得到标定力 (Fx_cal, Fy_cal)。凸包外退回到最近邻。"""
    fx = float(np.asarray(interp_fx(dx, dy)).flat[0])
    fy = float(np.asarray(interp_fy(dx, dy)).flat[0])

    if np.isnan(fx):
        fx = _nearest(dx, dy, interp_fx)
    if np.isnan(fy):
        fy = _nearest(dx, dy, interp_fy)

    return fx, fy


def _nearest(dx, dy, interp):
    """最近邻 fallback"""
    pts = interp.points
    vals = interp.values
    if pts is None or vals is None or len(pts) == 0:
        return 0.0
    dists = np.sum((pts - np.array([dx, dy])) ** 2, axis=1)
    idx = np.argmin(dists)
    return float(np.asarray(vals).ravel()[idx])


# ==================== CLI 入口 ====================
DEFAULT_SAVE_DIR = "/home/qcy/Project/data/2.PZT_tangential/weight/test"


def _resolve_path(arg: str) -> str:
    if arg.isdigit():
        return os.path.join(DEFAULT_SAVE_DIR, f"data_{arg}.csv")
    if os.path.sep not in arg and not arg.startswith("."):
        return os.path.join(DEFAULT_SAVE_DIR, arg)
    return arg


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("用法: python calibrate.py <csv_path|N|filename>")
        print("  python calibrate.py 1              → data_1.csv")
        print("  python calibrate.py data_42.csv    → SAVE_DIR/data_42.csv")
        sys.exit(1)

    csv_path = _resolve_path(sys.argv[1])
    out_path = os.path.join(os.path.dirname(csv_path), "cal_interp.pkl")

    try:
        interp_fx, interp_fy = build_from_csv(csv_path)
        save_interpolator(interp_fx, interp_fy, out_path)
    except Exception as e:
        print(f"  插值器构建失败: {e}")
        sys.exit(1)
