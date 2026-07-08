# CLAUDE.md

> 给Claude Code使用的工作手册。

## 项目画像
### 项目名: PZT_Hall
### 定位: 压阻阵列数据读取+数据处理+实时显示+数据保存
### 技术栈: python

## 架构分层

PZT_Hall/PZT_Hall/
├── main.py          #主函数，
├── realtime.py      #用于实时显示各种数据
├── data.py          #用于读取各种数据
├── fit.py     #用于标定数据
├── angle.py         #用于计算角度
├── COP.py           #用于计算COP(Center of Pressure)
├── table.py         #用于保存数据至csv文件
├── plot_static.py   #用于把数据绘制成静态图
├── calibrate.py     #查找表标定(lookup + discrete)
├── eskin_ffi.py     #libeskin_finger_sdk.so 的 ctypes 封装
├── libeskin_finger_sdk.so
└── CLAUDE.md        (给Claude Code使用的工作手册)

## 编码规范
### 命名约定


### 代码风格
- 把各个功能封装成函数，在各个主函数中都只是调用函数

## 注意事项
- 本项目只有压阻阵列，所以只需要处理压阻数据的代码

## ROS 2 适配(本项目相对参考项目的差异)

本项目嵌在 `rm_75_calibration` ROS 2 包中,与上层 `force_control_node` 配套工作。差异点:

### 集成方式
- **3 个 ROS 订阅器**(`main.py`):
  - `ForceDataSubscriber` 订阅 `/force_sensor_data`(`WrenchStamped`)→ 六维力数据来源
  - `PhaseSubscriber` 订阅 `/force_control_state`(`String`)→ 力控状态(进 HOLD 时 `latest_phase` 含 "HOLD")
  - `CopTriggerSubscriber` 订阅 `/cop_trigger` → 触发 `COP.trigger_cop_refine()`
- **1 个 ROS 发布器**(`main()` 退出时):
  - 发布 `/force_control_return_home` 通知 `force_control_node` 回原点
- **后台 ROS spin**:`ros2_spin(executor)` 在 `threading.Thread` 里跑 `SingleThreadedExecutor`

### 关键适配
- `MAIN_REFINE_REZERO_FORCE = False` 硬编码 —— 参考项目可以写本地 `sensor_force.zero_data`,本项目力来自 ROS 写不动;`force_data` 也不再有 `rezero` 概念
- `table.py` 的 `valid` 列(参考是 `CoP_state`)—— 本项目 `phase_node.latest_phase` 写入,作为力控到位的 ROS 信号,**不要改回 `CoP_state`**
- `data.py` 的 `PressureSensor` 仍从 `/dev/ttyUSB0` 读(本地 SDK);六维力不再走本地串口
- `fit_coefs_path` 默认为 `/home/qcy/Project/data/2.PZT_tangential/weight/png/fit_coefs.bin`;若 `MAIN_CAL_MODE = "fit"` 但文件不存在,自动 fallback 到 calibrate 引擎(无崩溃)

### 不动文件
- `COP.py`(用户刻意保留的 `COP_POST_INIT_*` 调参)
- `table.py`(`valid` 列必需)
- `angle.py`、`data.py`、`eskin_ffi.py`、`libeskin_finger_sdk.so`
- `example.py`、`realtime2.py`(备选 GUI,不在 ROS 数据路径上)

