#   /// 障碍物ID
int32 id
#   /// 障碍物类型
int8 type
#   /// 障碍物的状态
int32 status
#   /// 切入/切出的类型
int32 cut_in
#   /// 转向灯的状态
int32 blinker
#   /// 制动灯状态
bool brake_lights

int32 age
#   /// 指示障碍物在哪个车道
int32 lane
#   /// 障碍物长度 (m)
float32 length
#   /// 障碍宽度 (m)
float32 width
#   /// 障碍物高度（m）
float32 height
#   /// 障碍物位置x坐标 (m)
float32 x
#   /// 障碍物位置y坐标 (m)
float32 y
#   /// 障碍物航向 (rad)
float32 heading
#   /// 障碍物的x向速度 (m/sec)
float32 v_x
#   /// 障碍物的y向速度 (m/sec)
float32 v_y
#   /// 障碍物的加速度 (m/sec^2)
float32 accel_x
#   /// 障碍物的加速度 (m/sec^2)
float32 accel_y
#   /// 障碍物的角速度 (rad/sec)
float32 yaw_rate
#   /// 尺度的变化 (pix/sec)
float32 scale_change
# /// 障碍物绝对速度，沿着车身横轴的速度，单位：米/秒
float32 rel_v_x

