# /// 消息头
Header header
int32 sequence
int64 timestamp
# /// 驾驶模式
int8 driving_mode
# /// 紧急停止信号
int8 e_stop

# /// 方向盘控制状态
int8 eps_status
# /// 油门系统控制状态
int8 throttle_sys_status
# /// 制动系统控制状态
int8 ebs_status

# /// 方向盘角度信号是否有效
int8 steering_wheel_angle_valid
# /// 方向盘角度 (rad)
float32 steering_wheel_angle
# /// 方向盘转速有效位
int8 steering_wheel_speed_valid
# /// 方向盘转速 (rad/s)
float32 steering_wheel_speed
# /// 实际转向扭矩有效位
int8 steering_wheel_torque_valid
# /// 实际转向扭矩(N.m)
float32 steering_wheel_torque

# /// 车速信号是否有效
int8 v_valid
# /// 车辆速度,m/s
float32 v
# /// 车辆加速度信号是否有效
int8 a_valid
# /// 车辆加速度,m/s^2
float32 a
# /// 角速度有效位
int8 yaw_rate_valid
# /// 角速度,rad/s
float32 yaw_rate
# /// AX 有效位
int8 ax_valid
# /// AX(m/s^2)
float32 ax
# /// AY 有效位
int8 ay_valid
# /// AY(m/s^2)
float32 ay

# /// 左前轮速有效位
int8 wheel_speed_fl_valid
# /// 左前轮速(m/s)
float32 wheel_speed_fl
# /// 右前轮速有效位
int8 wheel_speed_fr_valid
# /// 右前轮速(m/s)
float32 wheel_speed_fr
# /// 左后轮速有效位
int8 wheel_speed_rl_valid
# /// 左后轮速(m/s)
float32 wheel_speed_rl
# /// 右后轮速有效位
int8 wheel_speed_rr_valid
# /// 右后轮速(m/s)
float32 wheel_speed_rr
# /// 左后2#轮速有效位
int8 wheel_speed_rl2_valid
# /// 左后2#轮速(m/s)
float32 wheel_speed_rl2
# /// 右后2#轮速有效位
int8 wheel_speed_rr2_valid
# /// 右后2#轮速(m/s)
float32 wheel_speed_rr2

# /// 驻车系统状态
int8 epb_status
# /// 当前档位
int8 gear
# /// 当前档位 Number
int8 gear_number
# /// 转向拨杆信号
int8 signal_turning_indicator
# /// 转向灯信号
int8 signal_turn_lamp
# /// 制动灯信号
int8 signal_brake_lamp
# /// 制动踏板深度（百分比）, invalid if less then 0
int8 brake_pedal_value
# /// 油门踏板深度（百分比）, invalid if less then 0
int8 acc_pedal_value

# /// 发动机转速有效位
int8 engine_speed_valid
# /// 发动机转速
float32 engine_speed
# /// 发动机转矩有效位
int8 engine_torque_valid
# /// 发动机转矩(N.m)
float32 engine_torque

# /// 总质量
int8 gross_weight_vaild
float32 gross_weight

int8 trailer_status
float32 trailer_l
float32 trailer_w
float32 trailer_h