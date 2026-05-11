# 终端 1：启动传感器 + 力控
ros2 launch rm_75_calibration calibration.launch.py

# 自定义轴和参数
ros2 launch rm_75_calibration calibration.launch.py \
  control_axes:=xz \
  z_target_force:=15.0 \
  x_target_force:=5.0 \
  x_start_delay:=5.0

# 终端 2：WSL 数据处理 + 实时显示（接收 ROS2 六维力数据）
python tang_7_12_Init_line_stable_COP_vec_cal_inter_realtime/main.py
